// Linkage between two bodies 

class Link {
  
  Body box1; //box
  Body box2; //box
  float speed;
  boolean head = false;
  RevoluteJointDef rjd;
  
  Link(Body a, Body b, boolean hd) {
    
    box1 = a;
    box2 = b;
    head = hd;
    
    rjd = new RevoluteJointDef();
    // need a world vector to the center of mass + to the location 
    /*
    Vec2 centA = a.getWorldCenter();
    Vec2 centB = b.getWorldCenter();
    Vec2 movA = new Vec2(box2d.scalarPixelsToWorld(20), 0);
    Vec2 movB = new Vec2(box2d.scalarPixelsToWorld(20), 0);
    */
    rjd.initialize(box1, box2, new Vec2(0,0)); // new Vec2(0,0) = dummy
    rjd.localAnchorA = new Vec2(2,0);
    rjd.localAnchorB = new Vec2(-2,0); 
    rjd.enableLimit = true;
    rjd.lowerAngle = -PI*2/3;
    rjd.upperAngle = PI*2/3;
    
    if (head) {
      rjd.enableMotor = true;
      rjd.maxMotorTorque = 10000;
      //rjd.motorSpeed = speed;
    }
    
    RevoluteJoint joint = (RevoluteJoint) box2d.world.createJoint(rjd);
  }
  
  public void set_motorSpeed(float spd) {
    rjd.motorSpeed = speed;
    speed = spd;
  }
}
