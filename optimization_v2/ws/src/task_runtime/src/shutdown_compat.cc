// Process-local compatibility for the locked Gazebo Sim 8.15.0 ABI.
// Serialize concurrent stop event emissions. Do not suppress signals, alter
// return codes, skip destructors, or intercept any motion/sensor function.
#include <dlfcn.h>
#include <pthread.h>
#include <cstdio>
#include <cstdlib>

static pthread_mutex_t stop_mutex = PTHREAD_MUTEX_INITIALIZER;
using Stop = void (*)(void *);
extern "C" void r01_serialized_stop(void *)
  asm("_ZN2gz3sim2v816SimulationRunner4StopEv");
extern "C" void r01_serialized_stop(void *self) {
  static Stop original = reinterpret_cast<Stop>(
    dlsym(RTLD_NEXT, "_ZN2gz3sim2v816SimulationRunner4StopEv"));
  if (!original) {
    std::fputs("R01_STOP_SYMBOL_NOT_FOUND\n", stderr);
    std::abort();
  }
  pthread_mutex_lock(&stop_mutex);
  std::fputs("R01_SERIALIZED_GAZEBO_STOP\n", stderr);
  original(self);
  pthread_mutex_unlock(&stop_mutex);
}
