#include "Locomotor.h"


using namespace std;


int main(int argc ,char* argv[]){

  Locomotor * single ;
  if(argc==1)
    single = Locomotor::getInstance();
  else
     single = Locomotor::getInstance( argv[1] , argv[2]);
  single->switchToKeyboard();

  //Locomotor loco;
  //   loco.goBackward(4);
  return 1;

}
