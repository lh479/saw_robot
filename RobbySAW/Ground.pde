// The Nature of Code
// <http://www.shiffman.net/teaching/nature>
// Spring 2012
// Box2DProcessing example

// A fixed boundary class

class Boundary {

  // Telling box2d that we are creating another body
  Body body;
  
  // A boundary is a simple rectangle with x,y,width,and height
  float x, y;
  float w, h;

  // Constructor
  Boundary(float x_,float y_, float w_, float h_) {
    x = x_;
    y = y_;
    w = w_;
    h = h_;
    
    // Step 1: Define Body
    BodyDef bd = new BodyDef();
    bd.type = BodyType.STATIC;
    bd.position.set(box2d.coordPixelsToWorld(x,y));
    
    // Step 2: Create Body
    body = box2d.createBody(bd);

    // Step 3: Create Shape
    PolygonShape ps = new PolygonShape();
    float box2dW = box2d.scalarPixelsToWorld(w/2);
    float box2dH = box2d.scalarPixelsToWorld(h/2);
    ps.setAsBox(box2dW, box2dH);

    // Step 4: Create Fixture
    FixtureDef fd = new FixtureDef();
    fd.shape = ps;
    // Parameters that affect physics
    fd.density = 1;
    fd.friction = 0.3;
    fd.restitution = 0.5;

   // Step 5: Attach Shape to Body with Fixture
    body.createFixture(fd);
}

  // Draw the boundary, if it were at an angle we'd have to do something fancier
  void display() {
    strokeWeight(1);
    stroke(0);
    fill(127);
    rect(x,y,w,h);
  }
}
