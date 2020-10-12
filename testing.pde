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
Boundary a;
Box b;
Box c;
Link lk_ab;
Link lk_bc;

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
  //links = new ArrayList<Box>();
  
  a = new Boundary(w_width/15*7, w_height/3, 48, 16);
  b = new Box(w_width/15*8, w_height/3, 48, 16);
  c = new Box(w_width/15*9, w_height/3, 48, 16);
  
  ground = new Boundary(w_width/2, w_height*4/5, g_w, g_h); // x, y, w, h

  lk_ab = new Link(a.body, b.body);
  lk_bc = new Link(b.body, c.body);
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
  
  a.display();
  b.display();
  c.display();
}
