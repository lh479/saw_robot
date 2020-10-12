//A rectangular box
// !! Generalized 
class Box {
  
  Body body;
  
  float x,y;
  float w,h;
  
  
  // Constructor: input in pixel (do not worry about /2)
  Box(float x_, float y_, float w_, float h_) {
    x = x_;
    y = y_;
    
    w = w_;  //16
    h = h_;  //16
    
    // Step 1: Define Body
    BodyDef bd = new BodyDef();
    bd.type = BodyType.DYNAMIC;
    bd.position.set(box2d.coordPixelsToWorld(x,y));  // initial position
    
    // Step 2: Create Body
    body = box2d.createBody(bd);
    
    // Step 3: Create Shape
    PolygonShape ps = new PolygonShape();
    float box2Dw = box2d.scalarPixelsToWorld(w/2);
    float box2Dh = box2d.scalarPixelsToWorld(h/2);
    ps.setAsBox(box2Dw, box2Dh); // the box dimension = (box2Dw*2)x(box2Dh)
    
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
  /*
  public Vec2 getWorldCenter() {
    return body.getWorldCenter();  
  }
  */
  
  void display() {
    Vec2 pos = box2d.getBodyPixelCoord(body);
    float a = body.getAngle();
    
    pushMatrix();
    translate(pos.x, pos.y);
    rotate(-a);
    fill(127);
    stroke(0);
    strokeWeight(2);
    rectMode(CENTER);
    rect(0,0,w,h);
    popMatrix();
  }
  
}
