
#ifndef __INCLUDE_DEBUG_HPP__
#define __INCLUDE_DEBUG_HPP__

#ifdef NDEBUG
  #define WITH_DEBUG(x) {  };
#else
  #define WITH_DEBUG(x) { x };
#endif // NDEBUG

#endif // __INCLUDE_DEBUG_HPP__
