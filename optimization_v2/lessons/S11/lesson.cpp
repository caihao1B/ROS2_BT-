// Teaching example: actual weighted CSR METIS call, followed by BFS per cluster.
#include <metis.h>
#include <vector>
#include <queue>
#include <iostream>
#include <stdexcept>
void valid_count(idx_t n,idx_t k){if(k<2 || k>n)throw std::invalid_argument("partition count out of range");}
int main() {
  idx_t n=36,k=3,ncon=1,cut=0;std::vector<idx_t>x{0},a,w,v(n),labels(n);
  for(idx_t i=0;i<n;i++) {
    v[i]=1+i%3; // service demand. It is independent of shared-edge penalty.
    for(idx_t j=0;j<n;j++)if((i/6==j/6 && abs(i-j)==1)||(i%6==j%6 && abs(i-j)==6)){a.push_back(j);w.push_back(10);}
    x.push_back(a.size());
  }
  valid_count(n,k);
  idx_t options[METIS_NOPTIONS];METIS_SetDefaultOptions(options);
  options[METIS_OPTION_SEED]=41;options[METIS_OPTION_CONTIG]=1;options[METIS_OPTION_NUMBERING]=0;
  int rc=METIS_PartGraphKway(&n,&ncon,x.data(),a.data(),v.data(),nullptr,w.data(),&k,nullptr,nullptr,options,&cut,labels.data());
  if(rc!=METIS_OK)throw std::runtime_error("METIS failed");
  for(idx_t c=0;c<k;c++) {
    int expected=0,start=-1,load=0;for(idx_t i=0;i<n;i++)if(labels[i]==c){expected++;start=i;load+=v[i];}
    if(start<0)throw std::runtime_error("empty cluster");
    std::vector<bool> seen(n,false);std::queue<int>q;q.push(start);seen[start]=true;int reached=0;
    while(!q.empty()){int i=q.front();q.pop();reached++;for(idx_t e=x[i];e<x[i+1];e++){auto j=a[e];if(labels[j]==c&&!seen[j]){seen[j]=true;q.push(j);}}}
    if(reached!=expected)throw std::runtime_error("disconnected cluster");
    std::cout<<"cluster "<<c<<" cells="<<expected<<" weighted_load="<<load<<"\n";
  }
  // Boundary example: k>n must be rejected before entering the C library.
  bool rejected=false;try{valid_count(n,n+1);}catch(const std::invalid_argument&){rejected=true;}
  if(!rejected)throw std::runtime_error("invalid count admitted");
  std::cout<<"PASS: actual METIS, three connected nonempty clusters; cut="<<cut<<". No route-time proof.\n";
}
