function Shape (sphere, flatOrRound,
                sphere2shape, shape2sphere, 
                child2parent, parent2child) {
    // sphere defines the underlying shape before distortion.
    this.sphere = sphere;

    // flatOrRound = 1:  flat shading.  = 2 : round shading.
    this.flatOrRound = flatOrRound;

    // sphere2shape is the distortion of the sphere to make the shape we want.
    this.sphere2shape = sphere2shape;
    this.shape2sphere = shape2sphere;

    // shape2child is the joint transformation.  Initially it is the
    // identity, meaning the joint is relaxed.  Changing shape2child
    // nods the head or bends the elbow.
    this.shape2child = new Mat();
    this.child2shape = new Mat();

    // child2parent is the position of the child with respect to the
    // parent when the joint is relaxed.  For example, the head is on
    // top of the body, not at its origin (center).
    this.child2parent = child2parent;
    this.parent2child = parent2child;

    // parent2world is the placement of the parent SHAPE in the world.
    // (Not the parent sphere.)
    this.parent2world = new Mat();
    this.world2parent = new Mat();

    // sphere2world is the placement of the sphere in the world.  It
    // gets bound to model2world in the shading program.
    this.sphere2world = new Mat();
    this.world2sphere = new Mat();
  
    // sMin is the smallest positive s for a ray q + s u hitting a face
    // of the sphere.  It is Double.POSITIVE_INFINITY if no face is hit.
    this.sMin = 1e9;

    // pMin is the point on the sphere at which the ray hits closest.
    this.pMin = new PV(true);

    // children contains the child shapes that are attached to this shape.
    this.children = [];

    this.setParent2World = function (parent2world, world2parent) {
        this.parent2world = parent2world;
        this.world2parent = world2parent;
        
        // EXERCISE 1:
        // Set this.sphere2world and this.world2sphere.
        // Call setParent2world for each child.  Be careful!  You don't
        // just pass along parent2world and world2parent.

        this.sphere2world = parent2world.times(this.child2parent.times(this.shape2child.times(this.sphere2shape)));
        this.world2sphere = this.shape2sphere.times(this.child2shape.times(this.parent2child.times(world2parent)));

        var shape2world = parent2world.times(this.child2parent.times(this.shape2child));
        var world2shape = this.child2shape.times(this.parent2child.times(world2parent));

        for (var i = 0; i < this['children']['length']; i++) {
            this.children[i].setParent2World(shape2world, world2shape)
        }

    }

    this.render = function (gl, program) {
        // EXERCISE 2A:
        // Set uniform variables model2world and world2modelT (T means transpose).
        // The sphere is the model.
	// Fix robot.html so it uses world2modelT where it should.

        var model2worldLoc = gl.getUniformLocation(program, 'model2world');
        var world2modelTLoc = gl.getUniformLocation(program, 'world2modelT');
        gl.uniformMatrix4fv(model2worldLoc, false, this.sphere2world.flatten());
        gl.uniformMatrix4fv(world2modelTLoc, false, this.world2sphere.transpose().flatten());


        this.sphere.render(gl, program, this.flatOrRound);
        
        // EXERCISE 2B:
        // Draw each child.
        
        for (var i = 0; i < this.children.length; i++) {
            this.children[i].render(gl, program)
        };
        
        // Test.
    }

    // Determine shape that is hit closest to front on the line segment
    // from front to back in world coordinates.  Set sMin and pMin for
    // that shape.  Return null if none.
    this. closestHit = function (front, back) {
        // EXERCISE 7:
        // Calculate q and u in sphere coordinates.
        var q = this.world2sphere.times(front);
        var u = this.world2sphere.times(back.minus(front));

        // Use this.sphere.closestHit to calculate this.sMin
        this.sMin = this.sphere.closestHit(q, u);

        // Calculate this.pMin
        this.pMin = q.plus(u.times(this.sMin));

    
        var closest = null;
        if (this.sMin != 1e9)
            closest = this;
        
	
        var closest = null;
        if (this.sMin != 1e9)
            closest = this;
        
        // Recurse for each child.  Take the one with closest hit of all.
        for (var i = 0; i < this.children.length; i++) {
            var childClosest = this.children[i].closestHit(front, back);
            if (childClosest != null && (closest == null || childClosest.sMin < closest.sMin)) {
                closest = childClosest
            }
        }
    
        return closest;
    }

    // Drag shape based on mouse position that corresponds to the line
    // segment from front to back in world coordinates.
    this.drag  = function (front, back) {
        var o = new PV(true); // in child coordinates

        // EXERCISE 9A:
        // v is the vector from the origin (in child coordinates) to pMin in
        // child coodrinates.
        // f and b are front and back in child coordinates.
        // Calculate of, fb, w, and the rotation of v into w.
        // Apply it to shape2child (which takes the place of model2rotated).
        var v = this.shape2child.times(this.sphere2shape.times(this.pMin)).minus(o);
        var f = this.parent2child.times(this.world2parent.times(front));
        var b = this.parent2child.times(this.world2parent.times(back));


        var of = f.minus(o);
        var fb = b.minus(f).unit();

        // (of + fb s) * fb = 0
        // of * fb + s = 0
        // s = - of * fb
        var s = -of.dot(fb);
        // w is the vector from the center to that closest point.
        var w = of.plus(fb.times(s));
        
        var r = v.magnitude();
        var l = w.magnitude();
        // If w is shorter than v,
        if (l <= r) {
            // Calculate how far we need to move along the line to be the
            // same distance from the center as v.
            var z = Math.sqrt(r*r - l*l);
            console.log("z " + z);
            // Move along the line that amount, in the same direction as v.
            if (v.dot(fb) > 0)
                w = w.plus(fb.times(z));
            else
                w = w.minus(fb.times(z));
        }
        else {
            // Otherwise scale w down to the magnitude of v.
            w.times(r/l);
        }
        
        // v is the x of a coordinate system.
        var vx = v;
        vx = vx.times(1/vx.magnitude());
        
        // w is the x of another coordinate system
        var wx = w;
        wx = wx.times(1/wx.magnitude());
        
        // Both systems should use v x w as their z direction.
        // So they are rotation about this z axis.
        var vz = vx.cross(wx);
        // Too short -- danger of divide by zero.
        if (vz.magnitude() < 1e-3)
            return;
        vz = vz.times(1/vz.magnitude());
        var wz = vz;
        
        // If you have x and z, you can get y.
        var vy = vz.cross(vx);
        var wy = wz.cross(wx);
        // Transforms [1 0 0 0]^T to vx
        var vMat = new Mat(vx, vy, vz);
        // Transforms [1 0 0 0]^T to wx
        var wMat = new Mat(wx, wy, wz);
        // Tranforms vx to [1 0 0 0]^T to wx.
        var vwMat = wMat.times(vMat.transpose());
        
        
        //EXERCISE 9B
        // Update orientation
        this.shape2child = vwMat.times(this.shape2child);
        this.child2shape = this.shape2child.transpose();
    }
}

function makeHead (gl, child2parent, parent2child) {
    var sphere = new Sphere(gl, 16, 2);
    var t = new PV(0, 1, 0, false);
    var sphere2shape = Mat.scale(0.5).times(Mat.translation(t));
    var shape2sphere = Mat.translation(t.minus()).times(Mat.scale(2));
    return new Shape(sphere, 2,
                     sphere2shape, shape2sphere,
                     child2parent, parent2child);
}

function makeRightArm(gl, child2parent, parent2child) {
    var sphere = new Sphere(gl, 16, 2);
    var t = new PV(4, -3, 0, false);
    var sphere2shape = Mat.scale(.5).times(Mat.translation(t));
    //var sphere2shape = Mat.scale(new PV(.5, .5, .5, false)).times(Mat.translation(t));
    var shape2sphere = Mat.translation(t.minus()).times(Mat.scale(2));
    //var shape2sphere = Mat.translation(t.minus()).times(Mat.scale(1/.5, 1/.5, 1/.5, false));
    return new Shape(sphere, 2,
                     sphere2shape, shape2sphere,
                     child2parent, parent2child);
}

function makePart(gl, sphere, s, t, t_other) {
	var sphere2shape = Mat.scale(s).times(Mat.translation(t));
	if (s instanceof PV) {
		var shape2sphere = Mat.translation(t.minus()).times(Mat.scale(new PV(1/s[0],1/s[1], 1/s[2], 1)));
	}
	else {
		var shape2sphere = Mat.translation(t.minus()).times(Mat.scale(1/s));
	}

	var child2parent = new Mat.translation(t_other);
	var parent2child = new Mat.translation(t_other.minus());

	return new Shape(
		sphere, 0,
		sphere2shape, shape2sphere,
		child2parent, parent2child
		);
}

function makeLeftArm(gl, child2parent, parent2child) {
    var sphere = new Sphere(gl, 16, 2);
    var t = new PV(-4, -3, 0, false);
    var sphere2shape = Mat.scale(.5).times(Mat.translation(t));
    var shape2sphere = Mat.translation(t.minus()).times(Mat.scale(2));
    return new Shape(sphere, 2,
                     sphere2shape, shape2sphere,
                     child2parent, parent2child);
}

function makeRightLeg(gl, child2parent, parent2child) {
    var sphere = new Sphere(gl, 16, 2);
    var t = new PV(2, -9, 0, false);
    var sphere2shape = Mat.scale(.5).times(Mat.translation(t));
    var shape2sphere = Mat.translation(t.minus()).times(Mat.scale(2));
    return new Shape(sphere, 2,
                     sphere2shape, shape2sphere,
                     child2parent, parent2child);
}

function makeLeftLeg(gl, child2parent, parent2child) {
    var sphere = new Sphere(gl, 16, 2);
    var t = new PV(-2, -9, 0, false);
    var sphere2shape = Mat.scale(.5).times(Mat.translation(t));
    var shape2sphere = Mat.translation(t.minus()).times(Mat.scale(2));
    return new Shape(sphere, 2,
                     sphere2shape, shape2sphere,
                     child2parent, parent2child);
}

function makeBody (gl) {
    // EXERCISE 3:
    // Test with simple and then change to complex.
    // Switch back and forth for tests.
    var simple = false;
    var body = null;
    if (simple) {
        var sphere = new Sphere(gl, 16, 8);
        var id = new Mat();
        body = new Shape(sphere, 2, id, id, id, id);
    }
    else {
        /*var sphere = new Sphere(gl, 4, 2);
        var sphere2shape = Mat.scale(new PV(1, 3, 1, true))
            .times(Mat.rotation(0, Math.PI/4));
        var shape2sphere = Mat.rotation(0, -Math.PI/4)
            .times(Mat.scale(new PV(1.0/1, 1.0/3, 1, true)));
        
        body = new Shape(sphere, 1, sphere2shape, shape2sphere,
                         new Mat(), new Mat());
      
        var t = new PV(0, 2, 0, false);
        var head2body = Mat.translation(t);
        var body2head = Mat.translation(t.minus());
        body.children.push(makeHead(gl, head2body, body2head));
        body.children.push(makeRightArm(gl, head2body, body2head));
        body.children.push(makeLeftArm(gl, head2body, body2head));
        body.children.push(makeRightLeg(gl, head2body, body2head));
        body.children.push(makeLeftLeg(gl, head2body, body2head));*/

        var body = makePart(gl,
        					new Sphere(gl, 16, 8, new PV(1, 0,0,0)),
        					new PV(.5,.5,.5,1), //s
        					new PV(0,0,0,0), //t, in relation to joint
        					new PV(0,-.5 ,0,0)); //t2, joint relation to body

        var bodymid = makePart(gl, 
        					new Sphere(gl, 16, 8, new PV(0,1,0,0)),
        					new PV(.25,.25,.25,1),
        					new PV(0,.3,0,0),
        					new PV(0,.5,0,0));

        var bodytop = makePart(gl, 
        					new Sphere(gl, 16, 8, new PV(0,0,1,0)),//latitude lines, longitude lines
        					new PV(.1,.1,.1,1),
        					new PV(0,.3,0,0),
        					new PV(0,.3,0,0));

        var bodyhatrectangle = makePart(gl, 
        					new Sphere(gl, 4, 2, new PV(0,0,0,0)),
        					new PV(.2,.1,.1,1),
        					new PV(0,.1,0,0),
        					new PV(0,.1,0,0));

         var bodyhatsquare = makePart(gl, 
         					new Sphere(gl, 4, 2, new PV(1,1,1,0)),
         					new PV(.1,.15,.1,1),
         					new PV(0,.1,0,0),
         					new PV(0,.1,0,0));

        var rightArm = makePart(gl,
        						new Sphere(gl, 16, 8, new PV(1,1,0,0)),
        						new PV(.4,.1,.1,1),
        						new PV(.5,0,0,0),
        						new PV(.1,0,0,0));

        var leftArm = makePart(gl,
        						new Sphere(gl, 16, 8, new PV(0,1,1,0)),
        						new PV(.4,.1,.1,1),
        						new PV(-.5,0,0,0),
        						new PV(-.1,0,0,0));

        body.children.push(bodymid);
        bodymid.children.push(bodytop);
        bodymid.children.push(rightArm);
        bodymid.children.push(leftArm);
        bodytop.children.push(bodyhatrectangle);
        bodyhatrectangle.children.push(bodyhatsquare);


    }
    return body;
  }


        

    
