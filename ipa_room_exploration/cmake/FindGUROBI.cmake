#### Taken from https://github.com/joschu/trajopt/blob/master/cmake/modules/FindGUROBI.cmake


# - Try to find GUROBI --> not a system package, needs to be searched separately
# Once done this will define
#  GUROBI_FOUND - System has Gurobi
#  GUROBI_INCLUDE_DIRS - The Gurobi include directories
#  GUROBI_LIBRARIES - The libraries needed to use Gurobi

if(GUROBI_INCLUDE_DIR)
	# variables already in cache
	set(GUROBI_FOUND TRUE)
	set(GUROBI_INCLUDE_DIRS "${GUROBI_INCLUDE_DIR}" )
	set(GUROBI_LIBRARIES "${GUROBI_LIBRARY};${GUROBI_CXX_LIBRARY}" )

else(GUROBI_INCLUDE_DIR)

	find_path(GUROBI_INCLUDE_DIR 
		NAMES	gurobi_c++.h
		PATHS	"$ENV{GUROBI_HOME}/include"
			"/Library/gurobi702/linux64/include"
	)

	# search for the used version of the Gurobi library
	find_library(GUROBI_LIBRARY 
		NAMES	gurobi
			gurobi45
			gurobi46
			gurobi50
			gurobi51
			gurobi52
			gurobi55
			gurobi56
			gurobi60
			gurobi70
		PATHS	"$ENV{GUROBI_HOME}/lib" 
			"/Library/gurobi702/linux64/lib"
	)

	find_library(GUROBI_CXX_LIBRARY 
		NAMES	gurobi_c++
			libgurobi_c++.a
		PATHS	"$ENV{GUROBI_HOME}/lib" 
			"/Library/gurobi702/linux64/lib"
	)

	if(GUROBI_LIBRARY AND GUROBI_CXX_LIBRARY AND GUROBI_INCLUDE_DIR)
		set(GUROBI_INCLUDE_DIRS "${GUROBI_INCLUDE_DIR}")
		set(GUROBI_LIBRARIES "${GUROBI_LIBRARY};${GUROBI_CXX_LIBRARY}")
		set(GUROBI_FOUND TRUE)
	else(GUROBI_LIBRARY AND GUROBI_CXX_LIBRARY AND GUROBI_INCLUDE_DIR)
		Message(STATUS "Gurobi has not been found")
	endif(GUROBI_LIBRARY AND GUROBI_CXX_LIBRARY AND GUROBI_INCLUDE_DIR)

	include(FindPackageHandleStandardArgs)
	# handle the QUIETLY and REQUIRED arguments
	find_package_handle_standard_args(GUROBI  DEFAULT_MSG
					GUROBI_LIBRARY GUROBI_CXX_LIBRARY GUROBI_INCLUDE_DIR)

	mark_as_advanced(GUROBI_INCLUDE_DIR GUROBI_LIBRARY GUROBI_CXX_LIBRARY)

endif(GUROBI_INCLUDE_DIR)
