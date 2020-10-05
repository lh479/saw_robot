import shiffman.box2d.*;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.joints.*;
import org.jbox2d.collision.shapes.*;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.contacts.*;

Box2DProcessing box2d;

// A list for all of our rectangles
ArrayList<Box> links;
Boundary ground;
Wave wave;

int w_width = 500; // window
int w_height = 500;
int g_w = w_width; // ground
int g_h = w_height/5*2;

void setup() {
  
  // Create Window
  size(500,500);
  
  // Create physics world
  box2d = new Box2DProcessing(this);
  box2d.createWorld();
  box2d.setGravity(0, -10); // real metrics
  
  // Create all the objects
  links = new ArrayList<Box>();
  for (int i = 1; i < 15; i++) {
    Box b = new Box(w_width/15*i, w_height/2, 48, 16);
    links.add(b);
  }
  ground = new Boundary(w_width/2, w_height*4/5, g_w, g_h); // x, y, w, h
  wave = new Wave(links);
}


void draw() {
  background(255);
  
  box2d.step();  // stepping to the next moment in time
  
  // Show where ground is 
  ground.display();
  /*
  if (mousePressed) {
    Box b = new Box(w_width/2, w_height/2);
    links.add(b);
  }
  */
  // Show where links are 
  
  for (Box b: links) {
    b.display();
  }
  
  //wave.display();

}
