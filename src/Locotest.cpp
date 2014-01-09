#include "Locomotor.h"


using namespace std;


int main(){
  Locomotor * single  = Locomotor::getInstance();
  single->switchToKeyboard();

  //Locomotor loco;
  //   loco.goBackward(4);
  return 1;

}
