/*This source code copyrighted by Lazy Foo' Productions (2004-2013)
  and may not be redistributed without written permission.*/

//The headers
#include"SoundPlayer.h"





SoundPlayer::SoundPlayer(){

  //Initialize all SDL subsystems
  SDL_Init( SDL_INIT_EVERYTHING );
  Mix_OpenAudio( 22050, MIX_DEFAULT_FORMAT, 2, 4096 );
  //Load the music
  music = Mix_LoadMUS( "../sound/Theme.mp3" );
  scratch = Mix_LoadWAV( "../sound/Coin.wav" );
  high = Mix_LoadWAV( "../sound/Jump.wav" );
  med = Mix_LoadWAV( "../sound/OutOfTime.wav" );
  low = Mix_LoadWAV( "../sound/StageCleared.wav" );
}

void SoundPlayer::pauseMusic(){

  //If there is no music playing
  if( Mix_PlayingMusic() != 0 )
    {

      if( Mix_PausedMusic() != 1 )
        {
          Mix_PauseMusic();
        }

    }
}


void SoundPlayer::playMusic(){

  if( Mix_PlayingMusic() == 0 )
    {
      Mix_PlayMusic( music, -1 );
    }
  else{
    if( Mix_PausedMusic() == 1 )
      {
        //Resume the music
        Mix_ResumeMusic();
      }
  }

}

void SoundPlayer::playSound(int i){

  switch(i){
  case 1:
    Mix_PlayChannel( -1, scratch, 0 ) ;
    break;

  case 2:
    Mix_PlayChannel( -1, low, 0 ) ;
    break;
  case 3:
    Mix_PlayChannel( -1, high, 0 ) ;
    break;
  case 4:
    Mix_PlayChannel( -1, med, 0 );
    break;
  }


}
SoundPlayer::~SoundPlayer()
{
  //Free the sound effects
  Mix_FreeChunk( scratch );
  Mix_FreeChunk( high );
  Mix_FreeChunk( med );
  Mix_FreeChunk( low );
  //Free the music
  Mix_FreeMusic( music );

  //Quit SDL_mixer
  Mix_CloseAudio();

  //Quit SDL
  SDL_Quit();
}


