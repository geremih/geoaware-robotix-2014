#include<iostream>
#include"SoundPlayer.h"

int main( int argc, char* args[] )
{
  SoundPlayer sp;
  int i;
  sp.playMusic();
  while(1){

    std::cin>>i;
    if(i ==-1)
      break;
               
    sp.playSound(i);
  }

  return 0;
}
