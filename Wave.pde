// drawing each link

class Wave {
  
  Body body;
  
  ArrayList<Box> links;
  Box box1;
  Box box2;
  
  // Constructor
  Wave(ArrayList<Box> links_) {
    
    // 1. Two bodies to hinge 
    links = links_;
    box1 = links.get(5);
    box2 = links.get(6);
    // PolygonShape can be generated from an array of vectors
    //PolygonShape link = new PolygonShape(); 
    //link.setAsBox(box2dW, box2dH);
    
    /*
    for (int i = 0; i < links.size(); i++) {
       if 
    }
    */
    
    // 2. Define the joint
    RevoluteJointDef rjd = new RevoluteJointDef();
    
    // 3. Configure the joint's properties
    // third parameter = anchor at the center of the body
    Vec2 A = new Vec2(0.75,0);
    Vec2 B = new Vec2(-0.75,0);
    
    rjd.localAnchorA = A;
    rjd.localAnchorB = B;
    rjd.initialize(box1.body, box2.body, rjd.localAnchorA);  
    /*rjd.enableLimit = true;
    rjd.lowerAngle = -PI/3;
    rjd.upperAngle = PI/3;
    */
    
    
    // 4. Create the joint
    RevoluteJoint joint = (RevoluteJoint) box2d.world.createJoint(rjd);
    
  }
  
  void display() {
    box1.display();
    box2.display();
  }
  
  void killBody() {
    box2d.destroyBody(body); 
  }
  
}
