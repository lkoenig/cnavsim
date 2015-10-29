

set(SRC
  main.cpp
  vessel.cpp
  vessel.hpp
  physics.cpp
  physics.hpp
  body.cpp
  body.hpp
  )

add_executable(navsim ${SRC})
target_link_libraries(navsim ${SDL_LIBRARY} ${EXTRA_LIBS})