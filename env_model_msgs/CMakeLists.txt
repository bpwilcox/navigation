cmake_minimum_required(VERSION 2.8.3)
project(env_model_msgs)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


find_package(catkin REQUIRED
        COMPONENTS
			message_generation
			std_msgs
        )



add_service_files(DIRECTORY srv
                  FILES
	FloatToInt.srv
	GetRayTrace.srv
	IsMapValid.srv
	GetValueAt.srv
	GetValueAtPos.srv	
	IntToFloat.srv)


generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    CATKIN_DEPENDS
		message_runtime
		std_msgs
)

