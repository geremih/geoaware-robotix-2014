#ifndef SOUNDPLAYER_H
#define SOUNDPLAYER_H

#include "SDL/SDL.h"
#include "SDL/SDL_mixer.h"


class SoundPlayer{

 public:
  SoundPlayer();
  ~SoundPlayer();
  void playSound(int i);
  void pauseMusic();
  void playMusic();
  
 private:
  
  Mix_Music *music;
  Mix_Chunk *scratch;
  Mix_Chunk *high;
  Mix_Chunk *med;
  Mix_Chunk *low ;
};

#endif
