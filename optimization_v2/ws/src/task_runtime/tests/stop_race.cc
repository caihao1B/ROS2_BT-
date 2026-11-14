// Real concurrent calls against an ABI-compatible test provider, never Gazebo.
#include <atomic>
#include <thread>
#include <chrono>
#include <cstdio>
#ifdef PROVIDER
std::atomic<int> active{0}, maximum{0};
extern "C" void stop(void*) asm("_ZN2gz3sim2v816SimulationRunner4StopEv");
extern "C" void stop(void*) {
  int now=++active, old=maximum.load();
  while(old<now && !maximum.compare_exchange_weak(old,now)) {}
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  --active;
}
extern "C" int peak(){return maximum;}
#else
extern "C" void stop(void*) asm("_ZN2gz3sim2v816SimulationRunner4StopEv");
extern "C" int peak();
int main(){std::atomic<int> ready{0};auto f=[&]{++ready;while(ready<2)std::this_thread::yield();stop(nullptr);};std::thread a(f),b(f);a.join();b.join();std::printf("%d\n",peak());}
#endif
