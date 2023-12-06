
float boundsX;
float boundsY;
int state;
float lastPosition[3];
float currentPosition[3];




void setup() {
  // put your setup code here, to run once:

}

void loop() {
  switch (state){
    case 0: 
      patrol();
      break;
    case 1:
      removeTree();
      break;
    case 2:
      getBack();
      break;
  }
}


void patrol(){
  if (treeDetected()){
    stop();
    lastPosition[0,1,2] = currentPosition[0,1,2]
    state++;
  }
  else{
      movement();
  }
}

void removeTree(){
  //turn to tree
  //move to tree
  //push tree out of bounds
}

void getBack(){
  //turn towards last position
  //drive to last postiion
  //turn to last postion
  state = 0;
}

void movement(){

}

bool treeDetected(){
  if (true){
    return true;
  }
}