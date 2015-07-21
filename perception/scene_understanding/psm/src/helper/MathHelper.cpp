#include "helper/MathHelper.h"

namespace ProbabilisticSceneRecognition {
 
  MathHelper::MathHelper()
  {
  }
  
  MathHelper::~MathHelper()
  {
  }

  void MathHelper::copy(plFloatVector& pFrom, boost::shared_ptr<Eigen::VectorXd>& pTo)
  {
    pTo.reset(new Eigen::VectorXd(pFrom.size())); 
    
    for(unsigned int i = 0; i < pFrom.size(); i++)
      (*pTo)[i] = pFrom[i];
  }
  
  void MathHelper::copy(plFloatMatrix& pFrom, boost::shared_ptr<Eigen::MatrixXd>& pTo)
  {
    pTo.reset(new Eigen::MatrixXd(pFrom.rows(), pFrom.cols())); 
    
    for(unsigned int y = 0; y < pFrom.rows(); y++)
      for(unsigned int x = 0; x < pFrom.cols(); x++)
      (*pTo)(y, x) = pFrom.at(y, x);
  }
  
}