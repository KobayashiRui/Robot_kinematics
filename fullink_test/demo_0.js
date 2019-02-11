tell("Demo 0 - Human bones");

addTarget(new THREE.Vector3(-40, 15, 0));
addTarget(new THREE.Vector3(30, 15, 0));
//addTarget(new THREE.Vector3(-8, -40, 0));
//addTarget(new THREE.Vector3( 8, -40, 0));

var startLoc = new FIK.V3();

var chain, basebone;

// 0 spine

chain = new FIK.Chain3D( 0xFFFF00 );
basebone = new FIK.Bone3D( startLoc, new FIK.V3( 0, 2, 0 ) );
chain.addBone( basebone );
chain.addConsecutiveRotorConstrainedBone( FIK.Y_AXE, 5, 30 );
chain.addConsecutiveRotorConstrainedBone( FIK.Y_AXE, 5, 30 );
chain.addConsecutiveRotorConstrainedBone( FIK.Y_AXE, 5, 30 );
chain.addConsecutiveRotorConstrainedBone( FIK.Y_AXE, 5, 30 );

solver.add( chain, targets[0].position, true );

// 1 left arm

chain = new FIK.Chain3D();
basebone = new FIK.Bone3D( new FIK.V3( 0, 20, 0 ), new FIK.V3( -10, 20, 0 ) );
//chain.addBone( basebone );
chain.addHingeBone(basebone)
//chain.addConsecutiveRotorConstrainedBone( FIK.X_NEG, 10, 90 );
//RotorConstrainedBone 
//chain.addConsecutiveRotorConstrainedBone( FIK.X_NEG, 10, 90 );//長さが10,角度90が上限
chain.addConsecutiveHingedBone( FIK.X_NEG, 10, 'global', FIK.Z_AXE, 90, 180, FIK.X_NEG );
//chain.addConsecutiveBone( FIK.X_NEG, 10 );
//chain.addConsecutiveBone( FIK.X_NEG, 10 );
//chain.addConsecutiveBone( FIK.X_NEG, 5 );

//chain.setRotorBaseboneConstraint( 'local', FIK.X_NEG, 0 );
solver.connectChain( chain, 0, 3, 'end', targets[1].position, true, 0x44FF44 );
// 2 right arm
