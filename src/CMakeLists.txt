

set(SRC
  vessel.cpp
  vessel.hpp
  physics.cpp
  physics.hpp
  body.cpp
  body.hpp
  )

add_library(navsim STATIC ${SRC})

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

add_subdirectory("cli")

if(ENABLE_GUI)
  add_subdirectory("gui")
endif()