

set(SRC
  main.cpp
  )

add_executable(navsim-cli ${SRC})
target_link_libraries(navsim-cli navsim)
