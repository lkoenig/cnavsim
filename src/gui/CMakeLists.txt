

set(SRC
  main.cpp
  )

add_executable(navsim-gui ${SRC})
target_link_libraries(navsim-gui navsim ${SDL2_LIBRARY} ${EXTRA_LIBS})
