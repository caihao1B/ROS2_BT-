#pragma once
namespace s14 {
// The PCA plane goes through its centroid. Anchoring it at a single cell's
// noisy lower quartile doubles the apparent height variation on flat ground.
template<class V> double planeHeight(const V& mean,const V& normal,const V& point){
 return mean.z()-(normal.x()*(point.x()-mean.x())+normal.y()*(point.y()-mean.y()))/normal.z();
}
}
