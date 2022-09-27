int r,g,b;

struct color{  
  int r;
  int g;
  int b;
  char name[8];
};

const color cor[7] = {
  {255,255,255, "nada"},
  {255,0  ,  0, "vermelho"}, //cor 1
  {0  ,255  ,  255, "laranja"},  //cor 2
  {255  ,  255,  0, "amarelo"},  //cor 3
  {0  ,255,  0, "verde"},    //cor 4
  {0  ,  0,255, "azul"},     //cor 5
  {0  ,  0,  0, "castanho"}  //cor 6
  };
int nearest_color = 0;

float distance(const int i){ // calcula a distancia de byte r,g,b a cor indicada no argumento
  float dist = sqrt(pow((r-cor[i].r),2) + pow((g-cor[i].g),2) + pow((b-cor[i].b),2));
  return dist;
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  r = 255;
  g = 10;
  b = 50;
  
  float min_dist = distance(0), cur_dist;
  for(int i = 0; i < 7;++i){
    cur_dist = distance(i);
    if(min_dist > cur_dist){
      nearest_color = i;
      min_dist = cur_dist;
    }
  }
  Serial.println(cor[nearest_color].name);
  while(1);
}

//funções necessarias //////////////////////////////////////////////////////////////////////////
