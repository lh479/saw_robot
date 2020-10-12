

class Link {
  
  Body box1; //box
  Body box2; //box
  
  Link(Body a, Body b) {
    
    box1 = a;
    box2 = b;
    
    RevoluteJointDef rjd_ab = new RevoluteJointDef();
    // need a world vector to the center of mass + to the location 
    /*
    Vec2 centA = a.getWorldCenter();
    Vec2 centB = b.getWorldCenter();
    Vec2 movA = new Vec2(box2d.scalarPixelsToWorld(20), 0);
    Vec2 movB = new Vec2(box2d.scalarPixelsToWorld(20), 0);
    */
    rjd_ab.initialize(box1, box2, new Vec2(0,0)); // (,,Vec2 anchor) -> anchor = dummy
    rjd_ab.localAnchorA = new Vec2(2,0);
    rjd_ab.localAnchorB = new Vec2(-2,0); 
    
    RevoluteJoint joint_ab = (RevoluteJoint) box2d.world.createJoint(rjd_ab);
  }
}
