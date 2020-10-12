// The Nature of Code
// <http://www.shiffman.net/teaching/nature>
// Spring 2012
// Box2DProcessing example

import shiffman.box2d.*;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.joints.*;
import org.jbox2d.collision.shapes.*;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.contacts.*;

Box2DProcessing box2d;

// A list for all of our rectangles
Boundary ground;
Boundary head;
Link lk;
ArrayList<Box> wave;
ArrayList<Link> links;

float w_width = 500; // window
float w_height = 500;
float g_w = w_width; // ground
float g_h = w_height/5*2;



void setup() {
  
  // Create Window
  size(500,500);
  
  // Create physics world
  box2d = new Box2DProcessing(this);
  box2d.createWorld();
  box2d.setGravity(0, -10); // real metrics
  
  // Create all the objects
  wave = new ArrayList<Box>();
  for (int i = 3; i < 10; i++) {  // 8 boxes including head
    Box b = new Box(w_width/15*i, w_height/3, 48, 16);
    wave.add(b);
  }
  // stationary head box
  head = new Boundary(w_width/15*13, w_height/3, 48, 16);
  
  links = new ArrayList<Link>();
  for (int i = 0; i < 6; i++) {
     lk = new Link(wave.get(i).body, wave.get(i+1).body, false);
     links.add(lk);
  }
  
  lk = new Link(wave.get(6).body, head.body, true); //link with head
  
  ground = new Boundary(w_width/2, w_height*4/5, g_w, g_h); // x, y, w, h
  
}

int time_count = 0;
float speed = 2000;

void draw() {
  background(255);
  /*
  time_count++;
  if (time_count % 1000 == 0) {
    lk.set_motorSpeed(-speed); 
  }
  */
  box2d.step();  // stepping to the next moment in time
 
  // Show where ground is 
  ground.display();
  
  head.display();
  for (Box b: wave) {
    b.display();
  }
}
