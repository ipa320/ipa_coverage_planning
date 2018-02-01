/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2016 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \author
 * Author: Joshua Hampp
 *
 * \date Date of creation: 2016
 *
 * \brief
 * classes for grid map generation in Voronoi manner
 *
 ****************************************************************/
 
#pragma once

#include <boost/shared_ptr.hpp>
#include <vector>
#include <queue>
#include <cmath>
#include <iostream>


#include <ipa_room_exploration/nanoflann.hpp>

#include <opencv2/opencv.hpp>


//debug includes
#include <ros/time.h>

#define APPROX_COVERED 3

//* Helper class containing 2d vector
struct Pos {
	int x_;		///< x position in pixels
	int y_;		///< y position in pixels
	
	/*!
	 * \brief setter constructor
	 */
	Pos(const int x, const int y) : x_(x), y_(y)
	{}
	
	inline bool operator==(const Pos &o) const {
		return x_==o.x_ && y_==o.y_;
	}
	
	/*!
	 * \brief add other position to this one
	 */
	inline void operator+=(const Pos &h) {
		x_ += h.x_;
		y_ += h.y_;
	}
	
	/*!
	 * \brief subtracts other position from this one
	 */
	inline void operator-=(const Pos &h) {
		x_ -= h.x_;
		y_ -= h.y_;
	}
	
	/*!
	 * \brief returns sum of two positions
	 */
	inline Pos operator+(const Pos &h) const {
		Pos c=*this;
		c+=h;
		return c;
	}
	
	/*!
	 * \brief returns difference of two positions
	 */
	inline Pos operator-(const Pos &h) const {
		Pos c=*this;
		c-=h;
		return c;
	}
	
	/*!
	 * \brief squared Euclidean distance to another postition
	 */
	inline int dist2(const Pos &h) const {
		return (x_-h.x_)*(x_-h.x_) + (y_-h.y_)*(y_-h.y_);
	}
	
	/*!
	 * \brief squared Euclidean distance of this position
	 */
	inline int dist2() const {
		return x_*x_ + y_*y_;
	}
	
};

std::ostream &operator<<(std::ostream &os, const Pos &p) {
	return os<<p.x_<<","<<p.y_;
}

//* Helper class containing meta information of grid space
struct Cell {
	Pos pos_;		///< position of cell in grid map
	int hops_;		///< number of hops needed to reach this cell from next obstacle
	int sx_;		///< relative movement in pixels (x)
	int sy_;		///< relative movement in pixels (y)
	int id_;		///< random identifier for identification of connected areas
	int id2_;		///< random identifier for internal processing (already processed...)
	
	typedef boost::shared_ptr<Cell> Ptr;
	
	/*!
	 * \brief default constructor
	 */
	Cell() : pos_(0,0), hops_(0), sx_(0), sy_(0), id_(-1), id2_(-1)
	{}
	/*!
	 * \brief constructor for defined position in grid map
	 */
	Cell(const int x, const int y) : pos_(x,y), hops_(1), sx_(0), sy_(0), id_(-1), id2_(-1)
	{}
	
	/*!
	 * \brief traverse to next cell by relative position increment
	 */
	inline void operator+=(const Pos &h) {
		hops_ += 1;
		sx_ += h.x_;
		sy_ += h.y_;
	}
	
	/*!
	 * \brief create a traversed cell by relative position increment
	 */
	inline Cell operator+(const Pos &h) const {
		Cell c=*this;
		c+=h;
		return c;
	}
	
	/*!
	 * \brief set relative movement and hops from another cell to this one
	 */
	inline void set(const Cell &h) {
		hops_ = h.hops_;
		sx_ = h.sx_;
		sy_ = h.sy_;
	}
	
	/*!
	 * \brief increases position, relative movement and hops from another cell to this one
	 */
	inline void operator+=(const Cell &h) {
		hops_ += h.hops_;
		pos_ += h.pos_;
		assert(sx_*h.sx_>=0);
		assert(sy_*h.sy_>=0);
		sx_ += h.sx_;
		sy_ += h.sy_;
	}
	
	/*!
	 * \brief returns summed up position, relative movement and hops from another cell and this one (rest of parameters are copied from this)
	 */
	inline Cell operator+(const Cell &h) const {
		Cell c=*this;
		c+=h;
		return c;
	}
	
	/*!
	 * \brief squared Euclidean distance of relative movement
	 */
	inline int dist2() const {
		return sx_*sx_ + sy_*sy_;
	}
	
	/*!
	 * \brief compares the Euclidean distances of relative movement
	 */
	inline bool operator<(const Cell &o) const { return dist2()<o.dist2(); }
	
	/*!
	 * \brief check for non-obstacles (hop counter > 0)
	 */
	inline operator bool() const { return hops_>0; }
	
};

//* Helper class for grid maps storing relevant meta information (Cell)
struct CellMap {
	std::vector<Cell> cells_;	///< ordered map of cells containing meta information
	int w_;		///< width in pixels
	int h_;		///< height in pixels
	
	/*!
	 * \brief constructor for creating a empty grid map of width*height
	 */
	CellMap(const int w, const int h) : cells_(w*h), w_(w), h_(h)
	{
		for(int x=0; x<w; x++)
			for(int y=0; y<h; y++)
				(*this)(x,y).pos_ = Pos(x,y);
	}
	
	/*!
	 * \brief returns the maximum of suqared Euclidean distances of cells (only used for visualization)
	 */
	int max() const 
	{
		int m=0;
		for(int x=0; x<w_; x++)
			for(int y=0; y<h_; y++)
				m = std::max((*this)(x,y).dist2(), m);
		return m;
	}
	
	/*!
	 * \brief check if coordinates are within valid range of map
	 */
	inline bool valid(const Pos &p) const {
		return (p.x_>=0 && p.x_<w_) && (p.y_>=0 && p.y_<h_);
	}
	
	/*!
	 * \brief access map by x/y coordinates in pixels
	 */
	inline Cell &operator()(const int x, const int y) {
		assert(x>=0 && x<w_);
		assert(y>=0 && y<h_);
		return cells_[y*w_+x];
	}
	
	/*!
	 * \brief access map by x/y coordinates in pixels
	 */
	inline const Cell &operator()(const int x, const int y) const {
		assert(x>=0 && x<w_);
		assert(y>=0 && y<h_);
		return cells_[y*w_+x];
	}
	
	/*!
	 * \brief access map by position in pixels
	 */
	inline Cell &operator[](const Pos &p) {
		return (*this)(p.x_, p.y_);
	}
	
};

struct CmpDist2Ref {
	Pos r;
	CmpDist2Ref(const Pos &r):r(r) {}
	
	inline bool operator()(const Pos &a, const Pos &b) { return a.dist2(r)<b.dist2(r);}
};

template<typename Type, typename Compare = std::less<Type> >
struct pless : public std::binary_function<Type *, Type *, bool> {
    bool operator()(const Type *x, const Type *y) const
        {
			if(x->dist2()==y->dist2()) {
				if(x->pos_.x_==y->pos_.x_)
					return x->pos_.y_>y->pos_.y_;
				return x->pos_.x_>y->pos_.x_;
			}
			return x->dist2()>y->dist2();
		}
};

//* Helper class for finding nearest neighbours
template <typename T>
struct PointCloud
{
	struct Point
	{
		T  x,y;
		
		Point() {}
		Point(const T &x, const T &y):x(x),y(y) {}
	};

	std::vector<Point>  pts;

	// Must return the number of data points
	inline size_t kdtree_get_point_count() const { return pts.size(); }

	// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
	inline T kdtree_distance(const T *p1, const size_t idx_p2,size_t /*size*/) const
	{
		const T d0=p1[0]-pts[idx_p2].x;
		const T d1=p1[1]-pts[idx_p2].y;
		return d0*d0+d1*d1;
	}

	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
	inline T kdtree_get_pt(const size_t idx, int dim) const
	{
		if (dim==0) return pts[idx].x;
		else return pts[idx].y;
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }

};

struct LineSegment {
	double length_;
	double effective_length_;
	std::vector<int> inds_;
	
	inline bool operator<(const LineSegment &o) const {
		return effective_length_>o.effective_length_;
	}
};

template<class T>
struct TSPItem {
	Pos in_, out_;
	cv::Point2f in_dir_, out_dir_;
	T *ref_;
	
	TSPItem() : in_(0,0), out_(0,0), ref_(NULL) {}
	
	TSPItem(const Pos &a, const Pos &b, const cv::Point2f &ad, const cv::Point2f &bd, T *ref):
	 in_(a), out_(b), in_dir_(ad), out_dir_(bd), ref_(ref)
	 {}
	
	TSPItem swap() const {
		return TSPItem(out_, in_, out_dir_, in_dir_, ref_);
	}
};

template<class T>
struct TSPTour {
	Pos start_;
	std::vector<T> items_;
	
	TSPTour(const Pos &s) : start_(s)
	{}
	
	inline size_t size() const {return 2*items_.size();}
	
	inline double costs() const {
		//TODO: include direction change as costs
		// depending on:
		//  * translation speed
		//  * rotation speed
		
		/*
		(1-last_dir.dot(dir)) 
		*/
		const double relation = (M_PI/0.2) * (0.05/0.3); //assumption: rotation_speed=0.2rad/s, translation_speed=0.3m/s, resolution=0.05m
		
		double c=0;
		Pos last = start_;
		cv::Point2f last_dir(0,0);
		for(size_t i=0; i<items_.size(); i++) {
			const double dist = std::sqrt(last.dist2(items_[i].in_));
			double rotation=0;
			if(dist>10) { //we replace last_dir
				Pos t = items_[i].in_-last;
				cv::Point2f interm(t.x_,t.y_);
				interm*= 1./dist;
				
				rotation = (1+interm.dot(items_[i].in_dir_));
				rotation+= (1-last_dir.dot(interm));
			}
			else {
				rotation = (1+last_dir.dot(items_[i].in_dir_));
			}
			
			c += dist +
				rotation*relation;
			last = items_[i].out_;
			last_dir = items_[i].out_dir_;
		}
		return c;
	}
	
	T get(const size_t i) const {
		assert(i/2<items_.size());
		
		if(i%2==0) return items_[i/2];
		return items_[i/2].swap();
	}
	
	void set(const size_t i, const T &d) {
		if(items_.size()<=i/2) items_.resize(i/2+1);
		
		if(i%2==0) items_[i/2] = d;
		else items_[i/2] = d.swap();
	}
};

template<class T>
struct TSPalgorithm {
	TSPTour<T> tour_;
	
	TSPalgorithm(const Pos &p) : tour_(p)
	{}

	// Do all 2-opt combinations
	void optimize()
	{ 
		// repeat until no improvement is made 
		bool improved = true;
		time_t start = time(NULL);
	 
		while ( improved )
		{
			improved=false;
			double best_distance = tour_.costs();
	 
			for ( int i = 0; i < tour_.size() - 1; i+=2 ) 
			{
				for ( int k = i + 1; k < tour_.size(); k++) 
				{
					TSPTour<T> new_tour(tour_.start_);
					swap2opt(new_tour, i, k );
	 
					const double new_distance = new_tour.costs();
	 
					if ( new_distance < best_distance ) 
					{
						// Improvement found so reset
						improved = true;
						tour_ = new_tour;
						best_distance = new_distance;
						
						//std::cout<<"new costs "<<new_distance<<std::endl;
						//Notify( tour_.costs() );
						
						i=tour_.size(); break; //go to start
					}
					
					/*if(time(NULL)-start>15) {
						std::cout<<"TIMEOUT!"<<std::endl;
						return;
					}*/
				}
			}
			
		}
	}

	void swap2opt(TSPTour<T> &new_tour, const int& i, const int& k ) 
	{
		// 1. take route[0] to route[i-1] and add them in order to new_route
		int c = 0;
		for (; c+1 <= i - 1; c+=2 )
		{
			new_tour.set( c, tour_.get( c ) );
		}
		 
		// 2. take route[i] to route[k] and add them in reverse order to new_route
		int dec = 0;//(i%2==1?2:0);
		for (; c<=k; c+=2 )
		{
			//std::cout<<c<<" "<<k<<" "<<dec<<std::endl;
			assert(dec<=k);
			new_tour.set( c, tour_.get( k - dec ) );
			dec+=2;
		}
	 
		// 3. take route[k+1] to end and add them in order to new_route
		for (; c < tour_.size(); c+=2 )
		{
			new_tour.set( c, tour_.get( c ) );
		}
	}

};

//* Helper function for finding an approximated shortest round trip for given set of points in consideration of starting positioin and coverage
template<class Points>
void connect(const Points &pts, std::vector<int> &out, const cv::Mat &coverage, const Pos &start, const int approx_cover_dist, const int min_segment_length=2, const int max_segment_length=5)
{
	if(pts.size()==0) return;
	else if(pts.size()==1) {
		out.push_back(0);
		out.push_back(0);
		return;
	}
	
	std::vector<int> used(pts.size(), 0);
	
	int current=0, num=0, last=0;
	
	//costs for already used/visited point
	const int USED_INC = 20;

	if(coverage.cols*coverage.rows>0) {
		for(size_t i=0; i<pts.size(); i++) {
			if(pts[i].x_<coverage.cols && pts[i].y_<coverage.rows && (used[i] =  -10*coverage.at<int8_t>(pts[i].y_, pts[i].x_)) )
				++num;
		}
	}
	
	///////////////////////////////////
	
	PointCloud<int> cloud;
	for(size_t i=0; i<pts.size(); i++)
		cloud.pts.push_back(PointCloud<int>::Point(pts[i].x_, pts[i].y_));
	
	// construct a kd-tree index:
	typedef nanoflann::KDTreeSingleIndexAdaptor<
		nanoflann::L2_Simple_Adaptor<int, PointCloud<int> > ,
		PointCloud<int>,
		2 /* dim */
		> my_kd_tree_t;

	my_kd_tree_t   index(2 /*dim*/, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );
	index.buildIndex();
	
	int query_pt[2];
	
	//1. build line segments
	std::vector<LineSegment> segs;
	//use first unused pt
	for(size_t j=0; j<used.size(); j++) {
		if(used[j]>0) continue;
		
		LineSegment seg;
		seg.length_ = 0;
		
		size_t nexts[4]={j};
		double last_lenghts[10];
		int num = 1;
		int nnn=0;
		while(num>0) {
			--num;
			size_t i = nexts[num];
			used[i] += USED_INC;
			
			//if(nnn%2==0) //only every 2nd point (to avoid odd patterns)
			{
				if(seg.inds_.size()>0)
					seg.length_ += std::sqrt(pts[seg.inds_.back()].dist2(pts[i]));
				last_lenghts[seg.inds_.size()%10] = seg.length_;
				seg.inds_.push_back(i);
			}
			++nnn;
			
			/*if(seg.length_>=max_segment_length) {
				seg.effective_length_ = seg.length_;
				segs.push_back(seg);

				seg = LineSegment();
				seg.length_ = 0;
			}*/
			
			//search for neighbouring pts
			query_pt[0] = pts[nexts[num]].x_;
			query_pt[1] = pts[nexts[num]].y_;
			
			std::vector<std::pair<size_t,int> >   ret_matches;
			nanoflann::SearchParams params;
			params.sorted = true;

			const size_t nMatches = index.radiusSearch(&query_pt[0], 5, ret_matches, params);
			//const size_t nMatches = index.knnSearch(&query_pt[0], 3 /*direct neighbours*/, ret_matches, params);
			//std::cout<<"nMatches "<<nMatches<<" ";
			int n = num;
			for (int j=(int)nMatches-1;j>=0;j--) {
				if(used[ret_matches[j].first]>0) continue;
				
				if(num>=4) { //finish segment
					//num = 1;
					break;
				}
				
				if(/*seg.inds_.size()==0 &&*/ num>0) { // only in one direction
					used[ret_matches[j].first] += USED_INC;
				}
				else {
					nexts[num] = ret_matches[j].first;
					++num;
				}
			}
			
			/*if(num==1 && n==1) { //if we had before 2 pts (front/back) and now only one again
				std::reverse(seg.inds_.begin(), seg.inds_.end());
			}*/
			if(seg.inds_.size()>10+min_segment_length) {
				Pos p1 = pts[seg.inds_[seg.inds_.size()-1]];
				Pos p2 = pts[seg.inds_[seg.inds_.size()-6]];
				Pos p3 = pts[seg.inds_[seg.inds_.size()-10]];
				Pos d1 = p2-p1;
				Pos d2 = p3-p2;
				if(d1.x_*d2.x_ + d1.y_*d2.y_<10) {
					std::vector<int> inds(seg.inds_.begin()+seg.inds_.size()-6, seg.inds_.end());
					double l = seg.length_-last_lenghts[(seg.inds_.size()-6)%10];
					seg.length_ = last_lenghts[(seg.inds_.size()-6)%10];
					seg.effective_length_ = seg.length_;
					seg.inds_.erase(seg.inds_.begin()+seg.inds_.size()-5, seg.inds_.end());
					
					if(seg.inds_.size()>0 && seg.effective_length_>=min_segment_length) segs.push_back(seg);
					
					seg.inds_=inds;
					seg.length_ = l;
				}
			}
			else if(seg.length_>80) {
				seg.effective_length_ = seg.length_;
				if(seg.inds_.size()>0 && seg.effective_length_>=min_segment_length) segs.push_back(seg);
				seg=LineSegment();
				seg.length_ = 0;
			}
		}
	
		seg.effective_length_ = seg.length_;
		if(seg.inds_.size()>0 && seg.effective_length_>=min_segment_length) segs.push_back(seg);
	}
	
	//3. do TSP for line segments
	TSPalgorithm<TSPItem<LineSegment> > opt(start);
	LineSegment segs_start={};
	for(size_t i=0; i<segs.size(); i++) {
		cv::Point2f dir_a(0,0), dir_b(0,0);
		if(segs[i].inds_.size()>1) {
			const size_t off = std::min((size_t)9, segs[i].inds_.size()-1);
			dir_a = cv::Point2f(pts[segs[i].inds_.front()].x_-pts[segs[i].inds_[off]].x_, pts[segs[i].inds_.front()].y_-pts[segs[i].inds_[off]].y_);
			dir_b = cv::Point2f(pts[segs[i].inds_.back()].x_-pts[segs[i].inds_[segs[i].inds_.size()-off-1]].x_, pts[segs[i].inds_.back()].y_-pts[segs[i].inds_[segs[i].inds_.size()-off-1]].y_);
			dir_a*= 1./cv::norm(dir_a);
			dir_b*= 1./cv::norm(dir_b);
			
			assert(!(dir_a!=dir_a));
			assert(!(dir_b!=dir_b));
		}
		opt.tour_.set(2*i, TSPItem<LineSegment>(pts[segs[i].inds_.front()], pts[segs[i].inds_.back()], dir_a, dir_b, &segs[i]));
	}
	
	opt.optimize();
	
	//4. create output
	Pos ll(start);
	for(size_t i=0; i<opt.tour_.items_.size(); i++) {
		if(opt.tour_.items_[i].in_==pts[opt.tour_.items_[i].ref_->inds_.back()])
			std::reverse(opt.tour_.items_[i].ref_->inds_.begin(), opt.tour_.items_[i].ref_->inds_.end());
			
		ll = opt.tour_.items_[i].out_;
		
		for(size_t j=0; j<opt.tour_.items_[i].ref_->inds_.size(); j++)
			out.push_back(opt.tour_.items_[i].ref_->inds_[j]);
	}
}

//* Class for building a modified Voronoi graph for a given grid map (also computes waypoints and areas)
class VoronoiMap {
	CellMap map_;					//< internal gridmap
	std::vector<Cell*> centers_;	//< central points definining connected areas (points which results from different obstacles -> local maxima)
	int max_track_width_;			//< width of a track in pixels
	int wall_offset_;				//< distance to wall for path generation in pixels
	bool single_room_;				//< if true the map is handled as one complete room
	
	typedef std::priority_queue<Cell*, std::vector<Cell*>, pless<Cell, std::greater<Cell> > > T_WAVE;
	
	enum {
		OCC=100, 	///< value for occupied cell in ROS gridmap
		FREE=0, 	///< value for free cell in ROS gridmap
		UNK=-1, 	///< value for unknown cell in ROS gridmap
		SET=3		///< value for internal handled cell
	};
	
	/*!
	 * \brief access operator for occopuancy grid map with width/height at position
	 */
	inline int8_t &operator()(int8_t *occ, const int w, const int h, const Pos &c) {
		return occ[c.y_*w+c.x_];
	}
	
	/*!
	 * \brief visit undhandled cells around given position and add them to our processing wave
	 */
	inline int add(int8_t *occ, const int w, const int h, const Pos &org, T_WAVE &wave) {
		const int NUM=4;
		static const Pos dirs[NUM] = {Pos(-1,0), Pos(0,-1), Pos(1,0), Pos(0,1)};
		
		const Cell &cur = map_[org];
				
		int n=0;
		for(int d=0; d<NUM; d++) {
			Pos p = org+dirs[d];
			Cell t =cur;
			t += dirs[d];
			if(map_.valid(p) && (*this)(occ,w,h,p)==FREE)
			{
				Cell *c = &map_[p];
				c->set(cur);
				*c += dirs[d];
				wave.push(c);
				(*this)(occ,w,h, p) = SET;
				++n;
			}
		}
		
		return n;
	}
	
	/*!
	 * \brief visit handled cells which are further away as from new position around given position and add them to our processing wave
	 */
	inline int add2(int8_t *occ, const int w, const int h, const Pos &org, T_WAVE &wave) {
		const int NUM=4;
		static const Pos dirs[NUM] = {Pos(-1,0), Pos(0,-1), Pos(1,0), Pos(0,1)};
		
		const Cell &cur = map_[org];
				
		int n=0;
		for(int d=0; d<NUM; d++) {
			Pos p = org+dirs[d];
			Cell t =cur;
			t += dirs[d];
			if(map_.valid(p) && ((*this)(occ,w,h,p)==SET||(*this)(occ,w,h,p)==UNK) && map_[p].dist2()>t.dist2())
			{
				Cell *c = &map_[p];
				c->set(cur);
				*c += dirs[d];
				wave.push(c);
				++n;
			}
		}
		
		return n;
	}
	
public:

	bool getSingleRoom() const {return single_room_;}
	void setSingleRoom(const bool b) {single_room_=b;}

	/*!
	 * \brief constructor for creating simple version of modified Voronoi map (without room segmenation...) for wall tracking purposes (potential field -> specified distance to wall)
	 */
	VoronoiMap(const int max_track_width, int8_t *occ, const int w, const int h) : map_(w,h), max_track_width_(max_track_width), wall_offset_(0), single_room_(false) {
		T_WAVE wave;
		
		for(int x=0; x<w; x++)
			for(int y=0; y<h; y++) {
				if(occ[y*w+x]==OCC) {
					const Pos p(x,y);
					wave.push(&map_[p]);
					(*this)(occ,w,h, p) = SET;
				}
				else if(occ[y*w+x]!=FREE&&occ[y*w+x]!=UNK)
					std::cout<<occ[y*w+x]<<std::endl;
			
			}
			
		while(wave.size()>0) {
			int added = add(occ,w,h, wave.top()->pos_, wave);
			
			if(added==0 && wave.top()->hops_>0 && wave.top()->dist2()>4)
				centers_.push_back(wave.top());
			
			wave.pop();
		}
	}
	
	/*!
	 * \brief constructor for creating "complete" modified Voronoi map (with room segmenation...) for maeander calculation (does not generate path by itself)
	 */
	VoronoiMap(int8_t *occ, const int w, const int h, const int max_track_width, const int merge_tracks=2, const bool single_room=false) : map_(w,h), max_track_width_(max_track_width), wall_offset_(0), single_room_(single_room) {
		T_WAVE wave;
		
		//std::cout<<"single_room "<<single_room<<" "<<max_track_width_<<std::endl;
		if(max_track_width_%2==1) --max_track_width_;
		
		for(int x=0; x<w; x++)
			for(int y=0; y<h; y++) {
				if(occ[y*w+x]==OCC) {
					const Pos p(x,y);
					wave.push(&map_[p]);
					(*this)(occ,w,h, p) = SET;
				}
				else if(occ[y*w+x]!=FREE&&occ[y*w+x]!=UNK)
					std::cout<<occ[y*w+x]<<std::endl;
			
			}
			
		while(wave.size()>0) {
			int added = add(occ,w,h, wave.top()->pos_, wave);
			
			if(added==0 && wave.top()->hops_>0 && wave.top()->dist2()>=max_track_width_*max_track_width_*2) {
				centers_.push_back(wave.top());
				if(single_room_) centers_.back()->id_=1;
			}
			
			wave.pop();
		}
		
		if(single_room) return;
		
		const int NUM=9;
		//static const Pos dirs[NUM] = {Pos(-1,0), Pos(0,-1), Pos(1,0), Pos(0,1), Pos(0,0)};
		static const Pos dirs[NUM] = {Pos(-1,-1), Pos(0,-1), Pos(1,-1), Pos(-1,0), Pos(1,0), Pos(-1,1), Pos(0,1), Pos(1,1), Pos(0,0)};
		
		PointCloud<int> cloud;
		for(size_t i=0; i<centers_.size(); i++)
			cloud.pts.push_back(PointCloud<int>::Point(centers_[i]->pos_.x_, centers_[i]->pos_.y_));
		
		// construct a kd-tree index:
		typedef nanoflann::KDTreeSingleIndexAdaptor<
			nanoflann::L2_Simple_Adaptor<int, PointCloud<int> > ,
			PointCloud<int>,
			2 /* dim */
			> my_kd_tree_t;

		my_kd_tree_t   index(2 /*dim*/, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );
		index.buildIndex();
		
		int query_pt[2];			
		std::vector<std::vector<Cell*> > ws_old;
		ws_old.resize(centers_.size());
		
		//start from central points with highest distance to next obstacle
		for(int i=centers_.size()-1; i>=0; --i) {
			
			//if already processed skip
			if(centers_[i]->id_>=0) continue;
			
			//start with this point
			std::vector<Cell*> ws;
			ws.push_back(centers_[i]);
			
			//iterate over all connected, unprocessed center points
			//for clustering
			for(size_t j=0; j<ws.size(); j++) {
				
				//if already processed skip
				if(ws[j]->id_>=0) continue;
					
				//distance to obstacle
				const int last_dist = std::sqrt(ws[j]->dist2());
				//track no.
				const int id = last_dist/(merge_tracks*max_track_width_);
				
				ws[j]->id_ = i+1;		//generate id for area
				
				//look now in direct neighbourhood for other center points
				query_pt[0] = ws[j]->pos_.x_;
				query_pt[1] = ws[j]->pos_.y_;
				
				std::vector<std::pair<size_t,int> >   ret_matches;
				nanoflann::SearchParams params;
				params.sorted = true;

				//const size_t nMatches = index.radiusSearch(&query_pt[0], 2*last_dist+2*max_track_width_, ret_matches, params);
					const size_t nMatches = index.radiusSearch(&query_pt[0], std::pow(last_dist+max_track_width_,2), ret_matches, params);
				
				int mid = id;
				for (size_t ii=0;ii<nMatches;ii++) {
					Cell *c = centers_[ret_matches[ii].first];
					if(c->id_>=0) continue; //already processed
					
					const int dist = std::sqrt(c->dist2());
					const int _id = dist/(merge_tracks*max_track_width_);
				
					//point would be within track no. --> clustering
					//so the clustered center points get the same id
					if(_id<=mid)
						ws.push_back(c);
					else break;
				}
			}
		}

		if(centers_.size()<1) return;

		//assign id to complete area around center point
		const int factor=merge_tracks;
		
		//biggest distance to obstacle
		const int first_dist2 = centers_[centers_.size()-1]->dist2();
		const int first_id = std::sqrt(first_dist2)/(factor*max_track_width_);
		
		std::vector<std::vector<Cell*> > wc;
		wc.resize(centers_.size());
		//seed points
		for(int i=0; i<centers_.size(); i++)
			wc[i].push_back(centers_[i]);
		
		//we need at most this much iterations
		//(wave should be empty and is used for border points)
		for(int p=0; p*p<=first_dist2*4; p++) {
			
			//start at biggest area
			for(int i=centers_.size()-1; i>=0; --i) {
				//distance to obstacle
				const int last_dist = std::sqrt(centers_[i]->dist2());
				//calc. track no.
				int id = last_dist/(factor*max_track_width_);
				
				//only process areas with at least this track no.
				if(id<first_id-(p*0.72)/(factor*max_track_width_))
					break;
					
				id = centers_[i]->id_; //get area id
				if(id<0) continue; //not processed by step before
					
				//do one loop through stack (initial one center point)
				std::vector<Cell*> w2;
				for(size_t j=0; j<wc[i].size(); j++) {
					
					//marked?
					//std::cout<<"SSS5 "<<wc[i][j]->id2_<<" "<<wc[i][j]->id_<<std::endl;
					if(wc[i][j]->id2_==99) {
						
						//if differnt area ids contract,
						//we mark this as border between two areas
						if(wc[i][j]->id_ != id || map_[wc[i][j]->pos_].dist2()<=0) {
							wc[i][j]->hops_=0;
							wc[i][j]->sx_=0;
							wc[i][j]->sy_=0;
							wave.push(wc[i][j]); //remember border
						}
						//otherwise it's the same area, we can skip
						
						continue;
					}
					
					wc[i][j]->id_ = id;  //assign area id
					wc[i][j]->id2_ = 99; //mark as visited
					
					//add surrounding points for next loop
					for(int d=0; d<NUM; d++) {
						Pos p = wc[i][j]->pos_+dirs[d];
						//if not an obstacle
						if(map_.valid(p))
							//if(map_[p].dist2()>0)
								w2.push_back(&map_[p]);
					}
				}
				
				//std::cout<<"SSS44 "<<w2.size()<<std::endl;
				
				//switch lists
				wc[i]=w2;
			}
			
		}
		
		//region growing starting from border points
		while(wave.size()>0) {
			int added = add2(occ,w,h, wave.top()->pos_, wave);
			//std::cout<<wave.size()<<" "<<wave.top()->dist2()<<std::endl;
			wave.pop();
		}
		
	}
	
	/*!
	 * \brief setter for distance to wall for path generation in pixels
	 */
	void setWallOffset(const int wall_offset) {
		wall_offset_ = wall_offset;
	}
	
	/*!
	 * \brief fills ROS gridmap with "get_simple" (assuming array is pre-allocated correctly in same dimensions as this map)
	 */
	void visualize_dist_map(int8_t *map, const int colission_radius) const {
		const double f = 127./std::sqrt(map_.max());
		
		for(int x=0; x<map_.w_; x++)
			for(int y=0; y<map_.h_; y++)
				map[y*map_.w_+x] = (int8_t)( get_simple(x,y, colission_radius)*255/150. );
	}
	
	/*!
	 * \brief value accessor for x/y coordinates in pixel with some intelligence (-1: out of bounds, 0: obstacle, 1000: if near to obstacles and minimum at half track width to obstacles)
	 */
	int get_simple(const int x, const int y, const int colission_radius) const
	{
		//boundary check
		if(x<1 || y<1 || x>=map_.w_-1 || y>=map_.h_-1)
			return -1;
			
		if(!map_(x,y).sx_ && !map_(x,y).sy_)
			return 0; //obstacle
			
		const double v = std::sqrt(map_(x,y).dist2())-max_track_width_/2.;
		if(v<colission_radius-max_track_width_/2.) return 1000;
		else if(v<0) return 2*std::abs(v)+1;
		
		return std::abs(v)+1;
	}
	
	/*!
	 * \brief value accessor for x/y coordinates in pixel with some intelligence (-1: out of bounds, 0: obstacle, 2222: if minimum in maeander and enough distance to next obstacle, otherwise 1000)
	 */
	int operator()(const int x, const int y) const
	{
		//boundary check
		if(x<1 || y<1 || x>=map_.w_-1 || y>=map_.h_-1)
			return -1;
			
		if(!map_(x,y).sx_ && !map_(x,y).sy_)
			return 0; //obstacle
			
		if(map_(x,y).hops_<=0) return 0;
		
		const float V = std::abs((float)fmod(std::sqrt((float)map_(x,y).dist2())-(float)wall_offset_, (float)max_track_width_)-(float)max_track_width_/2.f);
		if(V<0.45f) return 2222;
		
		const int xx = roundf(float(map_(x,y).sx_) / std::max(std::abs(map_(x,y).sx_), std::abs(map_(x,y).sy_)) );
		const int yy = roundf(float(map_(x,y).sy_) / std::max(std::abs(map_(x,y).sx_), std::abs(map_(x,y).sy_)) );
		
		const int xx2 = roundf(float(map_(x,y).sx_)*2 / std::max(std::abs(map_(x,y).sx_), std::abs(map_(x,y).sy_)) );
		const int yy2 = roundf(float(map_(x,y).sy_)*2 / std::max(std::abs(map_(x,y).sx_), std::abs(map_(x,y).sy_)) );
		
		const float v[5]={
			std::abs((float)fmod(std::sqrt((float)map_(x+xx,y+yy).dist2())-(float)wall_offset_, (float)max_track_width_)-(float)max_track_width_/2.f),
			V,
			std::abs((float)fmod(std::sqrt((float)map_(x-xx,y-yy).dist2())-(float)wall_offset_, (float)max_track_width_)-(float)max_track_width_/2.f),
			
			std::abs((float)fmod(std::sqrt((float)map_(x-xx2,y-yy2).dist2())-(float)wall_offset_, (float)max_track_width_)-(float)max_track_width_/2.f),
			std::abs((float)fmod(std::sqrt((float)map_(x+xx2,y+yy2).dist2())-(float)wall_offset_, (float)max_track_width_)-(float)max_track_width_/2.f)
		};
		
		//if(v[1]<v[0] && (v[1]<v[2]||(v[1]==v[2]&&v[1]<v[3])))
		//if(v[1]<v[0] && v[1]<=v[2])
		if(v[1]<v[3] && v[1]<v[4] && !(v[0]<v[4] && v[0]<v[1]) && !(v[2]<v[1] && v[2]<v[3]) )
		//if( (v[1]<v[0]||(v[1]<v[0]+0.1f&&v[1]<v[4])) && (v[1]<v[2]||(v[1]<v[2]+0.1f&&v[1]<v[3])))
			return 2222;
			
		return 1000;
			
		float vals[3][3];
		for(int xx=-1; xx<=1; xx++) {
			for(int yy=-1; yy<=1; yy++) {
				vals[xx+1][yy+1] = std::abs(fmod(std::sqrt(map_(x+xx,y+yy).dist2())-wall_offset_, max_track_width_)-max_track_width_/2.);
			}
		}
		
		int num=0;
		for(int i=0; i<3; i++)
			for(int j=0; j<3; j++)
				if(vals[i][j]<vals[1][1]) ++num;
				else if(vals[i][j]==vals[1][1]
					&& map_(x+i-1,y+j-1).hops_>map_(x,y).hops_) ++num;
				
		return (num==0 && map_(x,y).dist2()*12>=max_track_width_*max_track_width_)?2222:(map_(x,y).hops_>0?1000:0);
	}
	
	/*!
	 * \brief generate maeander using start position and coverage, simplifies path by minimum distance between points (in pixels) and deviation (distance) from a thought line (in pixel)
	 */
	template<class PathVector>
	void generatePath(PathVector &path, const cv::Mat &coverage, const int start_x, const int start_y, const int min_dist=40, const float dir_dev=0.1f) {
		std::vector<Pos> pts;
		size_t num=0;
		
		const int cid = map_(start_x,start_y).id_;
		
		for(int x=0; x<map_.w_; x++) {
			for(int y=0; y<map_.h_; y++) {
				//float v = std::abs(fmod(std::sqrt(map_(x,y).dist2())-wall_offset_, max_track_width_)-max_track_width_/2.);
				
				if( map_(x,y).dist2()>std::pow(wall_offset_,2) && (*this)(x,y) == 2222 && (single_room_||map_(x,y).id_==cid) ) {
					pts.push_back(Pos(x,y));
					++num;
				}
			}
		}
		
		//sort pts in relation to distance to start
		//-->circles wil be split near to start
		std::sort(pts.begin(), pts.end(), CmpDist2Ref(Pos(start_x, start_y)));
				
		num = 0;
			
		std::vector<int> out;
		connect(pts, out, coverage, Pos(start_x, start_y), APPROX_COVERED);
		
		//helper structure for optimized removal
		struct OptVal {
			struct OptValCompare {
				bool operator()(const typename std::list<OptVal>::iterator& lhs, const typename std::list<OptVal>::iterator& rhs) const {
					if(lhs->val==rhs->val) return lhs->val_sec<rhs->val_sec;
					return lhs->val<rhs->val;
				}
			};
			
			typedef std::multimap<typename std::list<OptVal>::iterator, char, OptValCompare> MM;
		
			double val;	//value defining costs if we remove the coresponding point from line
			double val_sec;
			int ind;	//index to order list (out) (double indexed to pts)
			typename MM::iterator mm;	//remember iterator to sorted multimap list
		};
		//function to update costs
		auto ValueFunc = [pts, out, this](typename std::list<OptVal>::iterator cur) {
			typename std::list<OptVal>::iterator b = cur;
			typename std::list<OptVal>::iterator a = cur;
			--b;
			++a;
			
			auto dx1 = pts[out[b->ind]].x_-pts[out[cur->ind]].x_;
			auto dy1 = pts[out[b->ind]].y_-pts[out[cur->ind]].y_;
			auto dx2 = pts[out[a->ind]].x_-pts[out[cur->ind]].x_;
			auto dy2 = pts[out[a->ind]].y_-pts[out[cur->ind]].y_;
			auto dx3 = pts[out[a->ind]].x_-pts[out[b->ind]].x_;
			auto dy3 = pts[out[a->ind]].y_-pts[out[b->ind]].y_;
			
			auto Dist = [](int dx, int dy) {return std::sqrt(std::pow(dx,2)+std::pow(dy,2));};
			auto l = Dist(dx1,dy1)+Dist(dx2,dy2);
			/*if(l<=2*this->max_track_width_ && std::abs(dx1*dy2 - dx2*dy1)<this->max_track_width_*this->max_track_width_/3)
				cur->val=0;
			else*/
				cur->val = (l-Dist(dx3,dy3))*Dist(dx3,dy3);
			//cur->val = (10000*(Dist(dx1,dy1)+Dist(dx2,dy2))-4)/Dist(dx3,dy3)-10000;
			//cur->val = std::abs(dx1*dy2 - dx2*dy1);
			cur->val_sec = Dist(dx3,dy3);
		};
		
		std::list<OptVal> vals;	//ordered list of line
		for(size_t i=0; i<out.size(); i++) {
			OptVal v;
			v.ind=i-1;
			v.val=0;
			v.val_sec=0;
			vals.push_back(v);
						}
			
		typename OptVal::MM vals_sorted;	//sorted list of line (costs)
		for(typename std::list<OptVal>::iterator it = ++vals.begin(); it!=--vals.end(); ++it) {
			ValueFunc(it);
			it->mm = vals_sorted.insert(std::pair<typename std::list<OptVal>::iterator, char>(it, 0));
			}
				
		const double THR=8;//std::pow(9,2);
		while(!vals_sorted.empty()) {
			typename std::list<OptVal>::iterator it = vals_sorted.begin()->first;
			if(it->val>=THR) break;
			
			//remove top (keep in mind, first and last point won't be removed!)
			typename std::list<OptVal>::iterator a=it, b=it;
			++a; --b;
			
			vals_sorted.erase(it->mm);
			if(a!=--vals.end()) vals_sorted.erase(a->mm);
			if(b!=vals.begin()) vals_sorted.erase(b->mm);
			vals.erase(it);
			
			//update values
			if(a!=--vals.end()) {
				ValueFunc(a);
				a->mm = vals_sorted.insert(std::pair<typename std::list<OptVal>::iterator, char>(a, 0));
			}
			if(b!=vals.begin()) {
				ValueFunc(b);
				b->mm = vals_sorted.insert(std::pair<typename std::list<OptVal>::iterator, char>(b, 0));
			}
		}
		
		path.resize(vals.size());
		size_t i=0;
		for(typename std::list<OptVal>::iterator it = vals.begin(); it!=vals.end(); ++it,++i) {
			path[i].x=pts[out[it->ind]].x_;
			path[i].y=pts[out[it->ind]].y_;
		}
		
	}
	
	/*!
	 * \brief gernates a direct path (2 points) to next uncovered area (e.g. next room to clean; shortest path)
	 */
	template<class PathVector>
	bool nextArea(PathVector &path, const cv::Mat &coverage, const int start_x, const int start_y) {
		path.resize(2);
		path[0].x = start_x;
		path[0].y = start_y;
		
		const int NUM=4;
		static const Pos dirs[NUM] = {Pos(-1,0), Pos(0,-1), Pos(1,0), Pos(0,1)};
			
		const int cid = map_(start_x,start_y).id_;
		Pos p(start_x, start_y);
		
		std::list<Pos> stack;
		stack.push_back(p);
		
		while(stack.size()>0) {
			p = stack.front();
			stack.pop_front();
			
			if( map_(p.x_,p.y_).id_!=cid && (*this)(p.x_,p.y_) == 2222 && 
					(p.x_>=coverage.cols || p.y_>=coverage.rows || !coverage.at<int8_t>(p.y_, p.x_) ) ) {
				path[1].x = p.x_;
				path[1].y = p.y_;
				
				return true;
			}
					
			int n=0;
			for(int d=0; d<NUM; d++) {
				Pos n = p+dirs[d];
				if(map_.valid(n) && map_(n.x_,n.y_).id_>=0 && map_(n.x_,n.y_).id2_!=333 ) {
					map_(n.x_,n.y_).id2_=333;
					stack.push_back(n);
				}
			}
		}
		
		return false;
	}
		
	/*!
	 * \brief generates a direct path (2 points) to next border (e.g. wall) which was noch covered yet (allows to search in global map or only in current area; shortest path)
	 */
	template<class PathVector>
	bool nextBorderPoint(PathVector &path, const cv::Mat &coverage, const int start_x, const int start_y, const bool stay_inside=false) {
		const int NUM=4;
		static const Pos dirs[NUM] = {Pos(-1,0), Pos(0,-1), Pos(1,0), Pos(0,1)};
			
		Pos p(start_x, start_y);
		
		std::list<Pos> stack;
		stack.push_back(p);
		
		path.resize(2);
		path[0].x = start_x;
		path[0].y = start_y;
		
		while(stack.size()>0) {
			p = stack.front();
			stack.pop_front();
			
			int d = map_(p.x_,p.y_).dist2();
			
			if( 4*d>=std::pow(3*max_track_width_-1,2) && 4*d<=std::pow(3*max_track_width_+1,2) && 
					(p.x_<0 || p.y_<0 || p.x_>=coverage.cols || p.y_>=coverage.rows || !coverage.at<int8_t>(p.y_, p.x_) ) ) {
				path[1].x = p.x_;
				path[1].y = p.y_;
				return true;
			}
					
			int n=0;
			for(int d=0; d<NUM; d++) {
				Pos n = p+dirs[d];
				if(map_.valid(n) && map_(n.x_,n.y_).id_>=0 && map_(n.x_,n.y_).id2_!=444 &&
				 (!stay_inside || map_(n.x_,n.y_).hops_>0) //TODO: check this statement
				) {
					map_(n.x_,n.y_).id2_=444;
					stack.push_back(n);
				}
			}
		}
		
		path.clear();
		return false;
	}
	
};
				
