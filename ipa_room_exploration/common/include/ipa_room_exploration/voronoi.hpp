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

//struct Edge;

struct Pos {
	int x_, y_;
	
	Pos(const int x, const int y) : x_(x), y_(y)
	{}
	
	inline void operator+=(const Pos &h) {
		x_ += h.x_;
		y_ += h.y_;
	}
	
	inline Pos operator+(const Pos &h) const {
		Pos c=*this;
		c+=h;
		return c;
	}
	
	inline int dist2(const Pos &h) const {
		return (x_-h.x_)*(x_-h.x_) + (y_-h.y_)*(y_-h.y_);
	}
	
};

struct Cell {
	Pos pos_;
	int hops_;
	float width_;
	int sx_, sy_;
	int id_, id2_;
	//boost::shared_ptr<Edge> edge_;
	
	typedef boost::shared_ptr<Cell> Ptr;
	
	Cell() : pos_(0,0), hops_(0), width_(0), sx_(0), sy_(0), id_(-1), id2_(-1)
	{}
	Cell(const int x, const int y) : pos_(x,y), hops_(1), width_(0), sx_(0), sy_(0), id_(-1), id2_(-1)
	{}
	
	inline void operator+=(const Pos &h) {
		hops_ += 1;
		//assert(sx_*h.x_>=0);
		//assert(sy_*h.y_>=0);
		sx_ += std::abs(h.x_);
		sy_ += std::abs(h.y_);
		//sx_ += h.x_;
		//sy_ += h.y_;
	}
	
	inline Cell operator+(const Pos &h) const {
		Cell c=*this;
		c+=h;
		return c;
	}
	
	inline void set(const Cell &h) {
		//assert(!*this);
		
		hops_ = h.hops_;
		sx_ = h.sx_;
		sy_ = h.sy_;
	}
	
	inline void operator+=(const Cell &h) {
		hops_ += h.hops_;
		pos_ += h.pos_;
		assert(sx_*h.sx_>=0);
		assert(sy_*h.sy_>=0);
		sx_ += h.sx_;
		sy_ += h.sy_;
	}
	
	inline Cell operator+(const Cell &h) const {
		Cell c=*this;
		c+=h;
		return c;
	}
	
	inline int dist2() const {
		return sx_*sx_ + sy_*sy_;
	}
	
	inline int dist_w(const Pos &p) const {
		//return std::sqrt((pos_.x_-p.x_)*(pos_.x_-p.x_) + (pos_.y_-p.y_)*(pos_.y_-p.y_));
		return std::max(std::abs(pos_.x_-p.x_), std::abs(pos_.y_-p.y_));
	}
	
	inline int dist2w(const Pos &p) const {
		return ((pos_.x_-p.x_)*(pos_.x_-p.x_) + (pos_.y_-p.y_)*(pos_.y_-p.y_));
	}
	
	inline int distMan() const {	//Manhatten distance
		return std::abs(sx_) + std::abs(sy_);
	}
	
	inline bool operator<(const Cell &o) const { return dist2()<o.dist2(); }
	
	inline operator bool() const { return hops_>0; }
	
};

struct CellMap {
	std::vector<Cell> cells_;
	int w_, h_;
	
	CellMap(const int w, const int h) : cells_(w*h), w_(w), h_(h)
	{
		for(int x=0; x<w; x++)
			for(int y=0; y<h; y++)
				(*this)(x,y).pos_ = Pos(x,y);
	}
	
	int max() const 
	{
		int m=0;
		for(int x=0; x<w_; x++)
			for(int y=0; y<h_; y++)
				m = std::max((*this)(x,y).dist2(), m);
		return m;
	}
	
	inline bool valid(const Pos &p) {
		return (p.x_>=0 && p.x_<w_) && (p.y_>=0 && p.y_<h_);
	}
	
	inline Cell &operator()(const int x, const int y) {
		assert(x>=0 && x<w_);
		assert(y>=0 && y<h_);
		return cells_[y*w_+x];
	}
	
	inline const Cell &operator()(const int x, const int y) const {
		assert(x>=0 && x<w_);
		assert(y>=0 && y<h_);
		return cells_[y*w_+x];
	}
	
	inline Cell &operator[](const Pos &p) {
		return (*this)(p.x_, p.y_);
	}
	
};

class CellGraph {
	Cell *cell_;
	std::vector<CellGraph*> ns_;
};


/*struct Edge {
	typedef boost::shared_ptr<Edge> Ptr;
	
	std::vector<Cell*> cells_;
	std::vector<Ptr> neighbours_;
	
	void add_neighbour(const Ptr &e) {
		for(size_t i=0; i<neighbours_.size(); i++) if(neighbours_[i]==e)
			return;
		neighbours_.push_back(e);
	}
	
	static bool compare_edges(const Edge::Ptr &e1, const Edge::Ptr &e2) {
		return e1->cells_.back()->dist2() < e2->cells_.back()->dist2();
	}
};*/

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

template<class Points>
void connect(const Points &pts, std::vector<int> &out, const cv::Mat &coverage, const Pos &start)
{
	if(pts.size()==0) return;
	else if(pts.size()==1) {
		out.push_back(0);
		out.push_back(0);
		return;
	}
	
	std::vector<int> used(pts.size(), 0);
	
	int current=0, num=0, last=0;
	
	const int USED_INC = 20;

	if(coverage.cols*coverage.rows>0) {
		std::cout<<"coverage "<<coverage.cols<<" "<<coverage.rows<<std::endl;
		for(size_t i=0; i<pts.size(); i++)
			if(pts[i].x<coverage.cols && pts[i].y<coverage.rows && (used[i] =  coverage.at<int8_t>(pts[i].x, pts[i].y)) )
				++num;
	}
	
	std::cout<<"connected: "<<num<<"/"<<pts.size()<<std::endl;
	
	for(size_t i=1; i<pts.size(); i++)
		if(start.dist2(Pos(pts[i].x,pts[i].y)) < start.dist2(Pos(pts[current].x,pts[current].y)))
			current = i;
	
	while(num<pts.size()) {
		
		double best_v=-1;
		int best_i=-1;
		
		for(size_t i=0; i<pts.size(); i++) {
			if(i==current) continue;
			
			const double dx1 = pts[i].x-pts[current].x;
			const double dy1 = pts[i].y-pts[current].y;
			const double dx2 = pts[current].x-pts[last].x;
			const double dy2 = pts[current].y-pts[last].y;
			
			const double dir = dx1*dx2 + dy1*dy2;
			
			//std::cout<<"dir "<<(dir*dir)/((dx1*dx1+dy1*dy1)*(dx2*dx2+dy2*dy2))<<" "<<dx1<<" "<<dy1<<" / "<<dx2<<" "<<dy2<<std::endl;
			
			const double dist2 = std::pow(pts[current].x-pts[i].x, 2) + std::pow(pts[current].y-pts[i].y, 2);
			const double v = (1.5-(dir*std::abs(dir))/((dx1*dx1+dy1*dy1)*(dx2*dx2+dy2*dy2)) )*(1+5*used[i]*used[i])*std::sqrt(dist2);
			if(best_i==-1 || v<best_v) {
				best_v = v;
				best_i = i;
			}
		}
		
		//std::cout<<"con "<<current<<" "<<best_i<<": "<<best_v<<"/"<<used[best_i]<<std::endl;
		
		if(!used[best_i])
			++num;
		used[best_i]+=USED_INC;
		out.push_back(current);
		last = current;
		current = best_i;
		out.push_back(current);
	}
}

class VoronoiMap {
	CellMap map_;
	//std::vector<Cell::Ptr> graph_;dfs
	//std::vector<Edge::Ptr> conns_;
	std::vector<Cell*> centers_;
	int max_track_width_;
	
	typedef std::priority_queue<Cell*, std::vector<Cell*>, pless<Cell, std::greater<Cell> > > T_WAVE;
	
	enum {OCC=100, FREE=0, UNK=-1, SET=3};
	
	inline int8_t &operator()(int8_t *occ, const int w, const int h, const Pos &c) {
		return occ[c.y_*w+c.x_];
	}
	
	inline int add(int8_t *occ, const int w, const int h, const Pos &org, T_WAVE &wave) {
		const int NUM=4;
		static const Pos dirs[NUM] = {Pos(-1,0), Pos(0,-1), Pos(1,0), Pos(0,1)};
		//const int NUM=8;
		//static const Pos dirs[NUM] = {Pos(-1,-1), Pos(0,-1), Pos(1,-1), Pos(-1,0), Pos(1,0), Pos(-1,1), Pos(0,1), Pos(1,1)};
		
		const Cell &cur = map_[org];
				
		int n=0;
		for(int d=0; d<NUM; d++) {
			Pos p = org+dirs[d];
			Cell t =cur;
			t += dirs[d];
			if(map_.valid(p) && ((*this)(occ,w,h,p)==FREE/* || map_[p].dist2()>t.dist2()*/))
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
	
	inline int add2(int8_t *occ, const int w, const int h, const Pos &org, T_WAVE &wave) {
		const int NUM=4;
		static const Pos dirs[NUM] = {Pos(-1,0), Pos(0,-1), Pos(1,0), Pos(0,1)};
		//const int NUM=8;
		//static const Pos dirs[NUM] = {Pos(-1,-1), Pos(0,-1), Pos(1,-1), Pos(-1,0), Pos(1,0), Pos(-1,1), Pos(0,1), Pos(1,1)};
		
		const Cell &cur = map_[org];
				
		int n=0;
		for(int d=0; d<NUM; d++) {
			Pos p = org+dirs[d];
			Cell t =cur;
			t += dirs[d];
			//std::cout<<p.x_<<"/"<<p.y_<<"  "<<map_[p].hops_<<" "<<cur.hops_<<"  "<<map_[p].dist2()<<" "<<t.dist2()<<std::endl;
			if(map_.valid(p) && (*this)(occ,w,h,p)==SET && map_[p].dist2()>t.dist2())
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
	VoronoiMap(int8_t *occ, const int w, const int h, const int max_track_width) : map_(w,h), max_track_width_(max_track_width) {
		T_WAVE wave;
		
		ros::Time ts_start = ros::Time::now();
		
		for(int x=0; x<w; x++)
			for(int y=0; y<h; y++) {
				if(occ[y*w+x]==OCC) {
					const Pos p(x,y);
					wave.push(&map_[p]);
					(*this)(occ,w,h, p) = SET;
					//add(occ,w,h, Pos(x,y), wave);
				}
				else if(occ[y*w+x]!=FREE&&occ[y*w+x]!=UNK)
					std::cout<<occ[y*w+x]<<std::endl;
			
			}
			
		while(wave.size()>0 /*&&wave.top()->hops_<depth*/) {
			
			//std::cout<<wave.top()->pos_.x_<<" "<<wave.top()->pos_.y_<<" "<<wave.top()->hops_+3<<std::endl;
		
			/*T_WAVE cp = wave;
			while(cp.size()>0) {
				std::cout<<cp.top()->dist2()<<" ";
				cp.pop();
			}
			std::cout<<wave.size()<<" "<<wave.top()->dist2()<<std::endl;*/
			
			int added=0;
			//if((*this)(occ,w,h, wave.top()->pos_)==FREE)
			{
				added = add(occ,w,h, wave.top()->pos_, wave);
				//(*this)(occ,w,h, wave.top()->pos_) = SET;
			} 
			
			if(added==0 && wave.top()->hops_>0)
			{
				/*std::cout<<wave.top()->pos_.x_<<" "<<wave.top()->pos_.y_<<std::endl;
				
				int min_hops=10000, max_hops=0, num=0;
				static const Pos dirs[4] = {Pos(-1,0), Pos(0,-1), Pos(1,0), Pos(0,1)};
				for(int d=0; d<4; d++)
					if(map_[wave.top()->pos_+dirs[d]].hops_==wave.top()->hops_)
						++num;
				
				for(int x=-1; x<=1; x++) {
					for(int y=-1; y<=1; y++) {
						if(!x&&!y) continue;
						const Cell &c = map_[wave.top()->pos_+Pos(x,y)];
						if(c) {
							max_hops = std::max(max_hops, c.dist2());
							//min_hops = std::min(min_hops, c.dist2());
						}
					}
				}
				
				std::cout<<num<<" "<<wave.top()->dist2()<<" "<<wave.top()->hops_<<" "<<max_hops<<" "<<min_hops<<std::endl;
				*/
				
				/*int num=0, num2=0;
				for(int x=-1; x<=1; x++) {
					for(int y=-1; y<=1; y++) {
						if(!x&&!y) continue;
						Pos p = wave.top()->pos_+Pos(x,y);
						if(!map_.valid(p)) continue;
						const Cell &c = map_[p];
						if(c && c.dist2()<=wave.top()->dist2()) {
							if(c.dist2()<wave.top()->dist2())
								++num2;
							else
								++num;
						}
					}
				}*/
				
				//if(num+num2==8 && num2>=6)
				if(wave.top()->dist2()>4) {
					//std::cout<<wave.top()->dist2()<<std::endl;
					
					centers_.push_back(wave.top());
			/*}
			
			//std::cout<<added<<std::endl;
			
			int surroundings = 0;
			if(added==0)
			{
				for(int x=-1; x<=1; x++)
					for(int y=-1; y<=1; y++)
						//if( map_(wave.top()->pos_.x_+x, wave.top()->pos_.y_+y) )
						if( occ[wave.top()->pos_.x_+x + (wave.top()->pos_.y_+y)*w]!=FREE )
							++surroundings;
			}
			
			std::cout<<surroundings<<std::endl;
			if(surroundings==9) {*/
				/*std::vector<Edge::Ptr> edges;
				for(int x=-1; x<=1; x++) {
					for(int y=-1; y<=1; y++) {
						if(!x&&!y) continue;
						const Cell &c = map_[wave.top()->pos_];
						if(c.edge_) edges.push_back(c.edge_);
					}
				}
				
				if(edges.size()==0) {
					edges.push_back(Edge::Ptr(new Edge()));
					conns_.push_back(edges.back());
				}
				
				if(edges.size()==1) {
					wave.top()->edge_ = edges.front();
					edges.front()->cells_.push_back(wave.top());
				}
				else {
					for(size_t i=0; i<edges.size(); i++) {
						for(size_t j=i+1; j<edges.size(); j++) {
							edges[i]->add_neighbour(edges[j]);
							//edges[j].add_neighbour(edges[i]);
						}
					}
				}*/
			}}
			
			wave.pop();
		}
		
		std::cout<<"took1: "<<(ros::Time::now()-ts_start).toSec()<<std::endl;
		
		//const int max_track_width = 6;
		//ROS_ASSERT(max_track_width%2==0);
		
		const int NUM=9;
		//static const Pos dirs[NUM] = {Pos(-1,0), Pos(0,-1), Pos(1,0), Pos(0,1), Pos(0,0)};
		static const Pos dirs[NUM] = {Pos(-1,-1), Pos(0,-1), Pos(1,-1), Pos(-1,0), Pos(1,0), Pos(-1,1), Pos(0,1), Pos(1,1), Pos(0,0)};
		
#if 1
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
		
		for(int i=centers_.size()-1; i>=0; --i) {
			if(centers_[i]->id_>=0) continue;
			
			std::vector<Cell*> ws;
			ws.push_back(centers_[i]);
			
			for(size_t j=0; j<ws.size(); j++) {
				if(ws[j]->id_>=0) continue;
					
				const int last_dist = std::sqrt(ws[j]->dist2());
				const int id = last_dist/max_track_width_;
				ws[j]->id_ = i;
				
				query_pt[0] = ws[j]->pos_.x_;
				query_pt[1] = ws[j]->pos_.y_;
				
				std::vector<std::pair<size_t,int> >   ret_matches;
				nanoflann::SearchParams params;
				params.sorted = true;

				const size_t nMatches = index.radiusSearch(&query_pt[0], last_dist, ret_matches, params);
				
				int mid = id;
				for (size_t i=0;i<nMatches;i++) {
					Cell *c = centers_[ret_matches[i].first];
					if(c->id_>=0) continue;
					
					const int dist = std::sqrt(c->dist2());
					const int _id = dist/max_track_width_;
				
					if(c->dist2()>4 && _id<=mid) {
						ws.push_back(c);
						mid = _id;
					}
					else break;
				}
			}
		}
		
#else
		for(int i=centers_.size()-1; i>=0; --i) {
			centers_[i]->id2_ = 1;
		}	
			
		std::vector<std::vector<Cell*> > ws_old;
		ws_old.resize(centers_.size());
		
		for(int i=centers_.size()-1; i>=0; --i) {
			if(centers_[i]->id_>=0) continue;
			
			std::vector<Cell*> ws;
			ws.push_back(centers_[i]);
			
			for(size_t j=0; j<ws.size(); j++) {
				if(ws[j]->id_>=0) continue;
					
				const int last_dist = std::sqrt(ws[j]->dist2());
				const int id = last_dist/max_track_width_;
				ws[j]->id_ = i;
				
				for(int d=0; d<NUM; d++) {
					Pos p = ws[j]->pos_+dirs[d];
					
					if(map_[p].id_>=0 || map_[p].id2_<0) continue;
					
					const int dist = std::sqrt(map_[p].dist2());
					const int _id = dist/max_track_width_;
				
					if(map_[p].dist2()>4 && _id<=id) ws.push_back(&map_[p]);
				}
			}
			
			/*for(size_t j=0; j<ws.size(); j++) {
				const int dist = std::sqrt(ws[j]->dist2());
				const int dist2= dist*dist;
				
				for(int x=-dist; x<=dist; x++) {
					for(int y=-dist; y<=dist; y++) {
						if(x*x+y*y>dist2) continue;
						
						Pos p = ws[j]->pos_+Pos(x,y);
						
						int dx = ws[j]->pos_.x_ - centers_[i]->pos_.x_;
						int dy = ws[j]->pos_.y_ - centers_[i]->pos_.y_;
						
						int dx2 = p.x_ - ws[j]->pos_.x_;
						int dy2 = p.y_ - ws[j]->pos_.y_;
						
						//if(dx*dx2 + dy*dy2 > 0) continue;
						
						if(map_[p].id_<0) map_[p].id_ = ws[j]->id_;
					}
				}
				
			}*/
			
			ws_old[i]=ws;
			//break;
		}
		
		/*for(size_t i=0; i<ws_old.size(); i++) {
			
			for(size_t j=0; j<ws_old[i].size(); j++) {
				const int dist = std::sqrt(ws_old[i][j]->dist2());
				const int dist2= dist*dist;
				
				for(int x=-dist; x<=dist; x++) {
					for(int y=-dist; y<=dist; y++) {
						if(x*x+y*y>dist2) continue;
						
						Pos p = ws_old[i][j]->pos_+Pos(x,y);						
						if(map_[p].id_<0) map_[p].id_ = ws_old[i][j]->id_;
					}
				}
				
			}
			
		}*/
#endif
#if 1
		const int factor=2;
		
		const int first_dist2 = centers_[centers_.size()-1]->dist2();
		const int first_id = std::sqrt(first_dist2)/(factor*max_track_width_);
		
		std::vector<std::vector<Cell*> > wc;
		wc.resize(centers_.size());
		for(int i=0; i<centers_.size(); i++)
			wc[i].push_back(centers_[i]);
		
		for(int p=0; p*p<=first_dist2*4; p++) {
			for(int i=centers_.size()-1; i>=0; --i) {
				const int last_dist = std::sqrt(centers_[i]->dist2());//centers_[i]->distMan();
				int id = last_dist/(factor*max_track_width_);
				
				if(id<first_id-(p*0.72)/(factor*max_track_width_))
					break;
					
				id = centers_[i]->id_;
				if(id<0) continue;
					
				std::vector<Cell*> w2;
				for(size_t j=0; j<wc[i].size(); j++) {
					if(wc[i][j]->id2_==99) {
						//const int _last_dist = std::sqrt(centers_[wc[i][j]->id_]->dist2());
						//if(wc[i][j]->id_ == id || _last_dist<=last_dist)
						{
						if(wc[i][j]->id_ != id) {
							wc[i][j]->hops_=0;
							wc[i][j]->sx_=0;
							wc[i][j]->sy_=0;
							wave.push(wc[i][j]);
							
							//std::cout<<"conc "<<_last_dist<<" "<<last_dist<<std::endl;
						}
						
				
						//if(_last_dist<last_dist)
							continue;
						}
					}
					wc[i][j]->id_ = id;
					wc[i][j]->id2_ = 99;
					
					for(int d=0; d<NUM; d++) {
						Pos p = wc[i][j]->pos_+dirs[d];
						if(map_.valid(p) && map_[p].dist2()>0) w2.push_back(&map_[p]);
					}
				}
				wc[i]=w2;
				
			}
		}
		
#endif
#if 0
		//for(int i=centers_.size()-1; i>=0; --i) {
		for(int i=0; i<centers_.size(); i++) {
			const int last_dist = std::sqrt(centers_[i]->dist2());//centers_[i]->distMan();
			
			const int number = (last_dist+max_track_width_-1)/max_track_width_; //number of possible tracks (rounded up)
			const float width = std::sqrt(centers_[i]->dist2());//  (last_dist/*+max_track_width_/2*/)/(float)number; //optimal track width
			
			std::cout<<"sxy "<<centers_[i]->sx_<<" "<<centers_[i]->sy_<<std::endl;
			
			map_[Pos(centers_[i]->pos_.x_, centers_[i]->pos_.y_)].width_= 111;
			//for(int j=-width; j<=width; j++)
			//	map_[Pos(centers_[i]->pos_.x_-centers_[i]->sx_*j/width, centers_[i]->pos_.y_+centers_[i]->sy_*j/width)].width_= 111;
			
			continue;
			
			//std::cout<<"w "<<width<<std::endl;
			
			if(width==0) continue;
			
			if(centers_[i]->width_<width) centers_[i]->width_= width;
			std::vector<Cell*> w, w2;
			w.push_back(centers_[i]);
			//while(hops>0) {
			while(w.size()>0) {
				
				for(size_t j=0; j<w.size(); j++) {
					int dist = w[j]->dist2();//w[j]->distMan();
					if(!dist) continue;
					
					for(int d=0; d<NUM; d++) {
						Pos p = w[j]->pos_+dirs[d];
//						if(map_[p].dist2()<dist && map_[p].width_==0) {
						//if(map_[p].hops_==hops && map_[p].width_==0) {
//						if(/*map_[p].dist2()<=dist &&*/ map_[p].width_<=width && map_[p].dist_w(centers_[i]->pos_)<=width && map_[p].dist2w(centers_[i]->pos_)>w[j]->dist2w(centers_[i]->pos_)) {
						if(/*map_[p].dist2()<=dist &&*/ map_[p].width_<width && map_[p].dist2w(centers_[i]->pos_)<=2*width*width ) {
//						if(map_[p].distMan()<dist && map_[p].width_==0) {
							map_[p].width_= width;
							w2.push_back(&map_[p]);
							//std::cout<<p.x_<<" "<<p.y_<<" "<<map_[p].dist2w(centers_[i]->pos_)<<"        "<<w[j]->pos_.x_<<" "<<w[j]->pos_.y_<<" "<<w[j]->dist2w(centers_[i]->pos_)<<std::endl;
						}
					}
				}
				
				w = w2;
				w2.clear();
			}
			
		}

#endif

		std::cout<<"took2: "<<(ros::Time::now()-ts_start).toSec()<<std::endl;
		
#if 1
		while(wave.size()>0) {
			int added = add2(occ,w,h, wave.top()->pos_, wave);
			//std::cout<<wave.size()<<" "<<wave.top()->dist2()<<std::endl;
			wave.pop();
		}
			
		std::cout<<"took3: "<<(ros::Time::now()-ts_start).toSec()<<std::endl;
#endif
		
	}
	
	/*void compute_meander() {
		//we start from the outmost intersections
		std::sort(conns_.begin(), conns_.end(), Edge::compare_edges);
		
		for(size_t i=0; i<conns_.size(); i++) {
			//calc. optimal width
			const double w = optimal from conns_[i].cells_.front()->dist2() and conns_[i].cells_.back()->dist2();
		}
		
	}*/
	
	void visualize_dist_map(int8_t *map) const {
		const double f = 127./std::sqrt(map_.max());
		
		for(int x=0; x<map_.w_; x++) {
			for(int y=0; y<map_.h_; y++) {
				//std::cout<<map_(x,y).width_<<" ";
				int m=1;
				//if(map_(x,y).edge_)
				//	m=-1;
				/*map[y*map_.w_+x] = (int8_t)(m*std::sqrt(map_(x,y).dist2())*f);
				//map[y*map_.w_+x] = (int8_t)(m*(map_(x,y).hops_)*127/map_.w_);
				if(m==-1) map[y*map_.w_+x] = -1;
				
				map[y*map_.w_+x] = (int8_t)((map_(x,y).width_-6)*255/2);	
				map[y*map_.w_+x] = (int8_t)(255-fmod(std::sqrt(map_(x,y).dist2())+max_track_width_/2,map_(x,y).width_)*255/map_(x,y).width_);*/
				
				map[y*map_.w_+x] = (int8_t)( (*this)(x,y)*255/4500. );
				//map[y*map_.w_+x] = (int8_t)((map_(x,y).width_)*255/max_track_width_);
				//map[y*map_.w_+x] = (int8_t)( std::abs(fmod(std::sqrt(map_(x,y).dist2()), max_track_width_)-max_track_width_/2.)*256/max_track_width_ );
			
			}
			//std::cout<<std::endl;
		}
	}
	
	int operator()(const int x, const int y) const
	{
//		if(x<0 || y<0 || x>=map_.w_ || y>=map_.h_)
//			return -1;
		//return map_(x,y).dist2();
		//return 1000-fmod(std::sqrt(map_(x,y).dist2()),map_(x,y).width_)*1000/map_(x,y).width_;
		
		if(!map_(x,y).sx_ && !map_(x,y).sy_)
			return 0; //obstacle
			
		//return map_(x,y).id_*1300;
		
		//if(std::abs(fmod(std::sqrt(map_(x,y).dist2()), max_track_width_)-max_track_width_/2.)>0.75) return 0;
		
		if(x<1 || y<1 || x>=map_.w_-1 || y>=map_.h_-1)
			return -1;
			
		float vals[3][3];
		for(int xx=-1; xx<=1; xx++) {
			for(int yy=-1; yy<=1; yy++) {
				vals[xx+1][yy+1] = std::abs(fmod(std::sqrt(map_(x+xx,y+yy).dist2()), max_track_width_)-max_track_width_/2.);
			}
		}
		
		int num=0;
		for(int i=0; i<3; i++)
			for(int j=0; j<3; j++)
				/*if(vals[i][j]<=vals[1][1]
					|| map_(x+i-1,y+j-1).hops_>map_(x,y).hops_) ++num;*/
				if(vals[i][j]<vals[1][1]) ++num;
				else if(vals[i][j]==vals[1][1]
					&& map_(x+i-1,y+j-1).hops_>map_(x,y).hops_) ++num;
				
		return (num==0 && map_(x,y).dist2()*12>=max_track_width_*max_track_width_)?2222:(map_(x,y).hops_>0?1000:0);
			
		return map_(x,y).id2_!=111&&std::abs(fmod(std::sqrt(map_(x,y).dist2()), max_track_width_)-max_track_width_/2.)>0.75?0:2222;
		
		return //map_(x,y).id_<0?0:map_(x,y).id_*13;
			//map_(x,y).width_?1000*(int(map_(x,y).width_)/(max_track_width_)):4400;
			/*20**/(210*(max_track_width_+1))*( int(map_(x,y).width_)/(2*max_track_width_) + 1 )
			//- 510*(int(std::sqrt(map_(x,y).dist2()))/max_track_width_)
			+ 1000-std::abs(fmod(std::sqrt(map_(x,y).dist2()), max_track_width_)-max_track_width_/2.)*1500/max_track_width_;
		
		return 1000-std::abs(fmod(std::sqrt(map_(x,y).dist2()), max_track_width_)-max_track_width_/2.)*1500/max_track_width_;
		
		return 1000-fmod(std::sqrt(map_(x,y).dist2())+max_track_width_/2,max_track_width_)*1000/max_track_width_;
		//TODO: dynamic width -> optimize (see below)
		//return 1000-fmod(std::sqrt(map_(x,y).dist2())+max_track_width_/2,map_(x,y).width_)*1000/map_(x,y).width_;
	}
	
	template<class PathVector>
	void generatePath(PathVector &path, const cv::Mat &coverage, const int start_x, const int start_y) const {
		PointCloud<int> cloud;
		size_t num=0;
		
		const int cid = map_(start_x,start_y).id_;
		
		for(int x=0; x<map_.w_; x++) {
			for(int y=0; y<map_.h_; y++) {
				if( (*this)(x,y) == 2222 && map_(x,y).id_==cid) {
					cloud.pts.push_back(PointCloud<int>::Point(x,y));
					++num;
				}
			}
		}
		
		num = 0;
			
		std::vector<int> out;
		connect(cloud.pts, out, coverage, Pos(start_x, start_y));
		path.resize(out.size());
		
		Pos last(0,0);
		for(size_t i=0; i<out.size(); i+=2)
		{
			bool replace = false;
			
			Pos t(cloud.pts[out[i+1]].x-cloud.pts[out[i]].x, cloud.pts[out[i+1]].y-cloud.pts[out[i]].y);
			if(i) {
				if(t.x_*last.y_ == t.y_*last.x_)
					replace = true;
			}
			last = t;
			
			if(replace) {
				path[num-1].x = cloud.pts[out[i+1]].x;
				path[num-1].y = cloud.pts[out[i+1]].y;
			}
			else {
				path[num].x = cloud.pts[out[i]].x;
				path[num].y = cloud.pts[out[i]].y;
				++num;
				
				path[num].x = cloud.pts[out[i+1]].x;
				path[num].y = cloud.pts[out[i+1]].y;
				++num;
			}
		}
		
		if(out.size()>0) {
			if(path.size()<num+2)
				path.resize(num+2);
				
			size_t i=out.size()-1;
			
			path[num].x = cloud.pts[out[i]].x;
			path[num].y = cloud.pts[out[i]].y;
			++num;
			
			Pos p(cloud.pts[out[i]].x, cloud.pts[out[i]].y);
			int dist=-1;
			for(int x=0; x<map_.w_; x++) {
				for(int y=0; y<map_.h_; y++) {
					if( (*this)(x,y) == 2222 && map_(x,y).id_!=cid && (dist<0||p.dist2(Pos(x,y))<dist) ) {
						dist = p.dist2(Pos(x,y));
						path[num].x = x;
						path[num].y = y;
					}
				}
			}
			++num;
		}
		
		path.resize(num);
	}
	
	template<class PathVector>
	void generatePath(PathVector &path, const cv::Mat &coverage) const {
		const int start_x=0;
		const int start_y=0;
		
		std::vector<PointCloud<int> > clouds;
		clouds.resize(centers_.size());
		size_t num=0;
		
		for(int x=0; x<map_.w_; x++) {
			for(int y=0; y<map_.h_; y++) {
				if( (*this)(x,y) == 2222 ) {
					clouds[map_(x,y).id_].pts.push_back(PointCloud<int>::Point(x,y));
					++num;
				}
			}
		}
		
		path.resize(num);
		num = 0;
		
		for(size_t k=0; k<clouds.size(); k++) {
			PointCloud<int> &cloud = clouds[k];
			if(cloud.pts.size()<1) continue;
			
			std::vector<int> out;
			connect(cloud.pts, out, coverage, Pos(start_x, start_y));
			
			for(size_t i=0; i<out.size(); i++)
			{
			   //std::cout << out[i] << " ";
			   
			   if(path.size()<num+1)
				path.resize(num+1);
			   
				path[num].x = cloud.pts[out[i]].x;
				path[num].y = cloud.pts[out[i]].y;
				++num;
			}
		}
	}
};
				
