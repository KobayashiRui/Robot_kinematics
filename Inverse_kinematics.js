var Link_list = [];
var name2link = {};
var link_num = 3;
var flag = 0;
var lambda = 0.2;
var Make_link_list = [[[0],[0],[1]],[[0],[1],[0]],[[0],[1],[0]]]

Make_world();
var buf_link_name = "world_link";
for(let i=0; i < link_num; i++){
    let link_name = "link_"+i;
    let link_length = [[0],[0],[1]];
    let axis = Make_link_list[i];//[[0],[0],[1]];
    Make_link(link_name,buf_link_name,link_length,axis,20);
    name2link[link_name] = Link_list[i+1];
   // Make_Controler(link_name);
    buf_link_name = link_name;
}

var target_rot_data = Rpy2rot(0,40,0);
var Target_data = {pos:[[1.5],[1.5],[2]],rot:target_rot_data};

for(let I = 0; I < 1000; I++){
console.log("Number:"+I);
ForwardKinematics(Link_list);
//console.log(JSON.stringify(Link_list));
console.log("順運動学終了");
var Jaco = CalcJacobian(Link_list);
//console.log(JSON.stringify(Jaco.valueOf()));
var Error_data = CalcVWerr(Target_data,Link_list[Link_list.length-1]);
console.log("end??");
var Error_norm = Error_data.valueOf();
console.log(math.norm(math.reshape(Error_norm,[6])));
if(math.norm(math.reshape(Error_norm,[6])) < 0.00001){
  console.log("end");
  break;
}
console.log(JSON.stringify(Error_data));
console.log("ヤコビの逆行列")
if(Jaco.length != Jaco[0].length){
var Jaco_inv = pinv(Jaco);  
}
//console.log(JSON.stringify(Jaco_inv.valueOf()));
console.log("error");
console.log(JSON.stringify(Error_data.valueOf()));
//console.log(JSON.stringify(math.usolve(Jaco,Error_data)));
var dq = math.multiply(math.multiply(Jaco_inv,Error_data),lambda).valueOf();
//console.log(JSON.stringify(dq));
for(let i = 1; i < Link_list.length; i++){
  Link_list[i].angle = Link_list[i].angle + (dq[i-1][0]*180/Math.PI);
}

//console.log(JSON.stringify(Jaco));
console.log(JSON.stringify(Link_list));
//console.log(JSON.stringify(Error_data))
//Show_arm();
}

Show_arm()


//原点位置の作成
function Make_world(){
    let World_link = {link_name:"world_link",pos:[[0],[0],[0.5]],rot:[[1,0,0],[0,1,0],[0,0,1]]};
    Link_list.push(World_link);   
}

//リンクデータの作成
function Make_link(name,mather_name,link_length,axis,angle){
    let link = {link_name:name,mather_name:mather_name,link_length:link_length,axis:axis,angle:angle};
    Link_list.push(link);
}

//ロドリゲスの公式
function Rodrigues(axis,angle){//axis is 3 * 1 matrix angle is degree
    let R = [[0,0,0],[0,0,0],[0,0,0]];
    let rad = angle * Math.PI / 180;
    let C_ = Math.cos(rad);
    let S_ = Math.sin(rad);
    let n_x = axis[0][0];
    let n_y = axis[1][0];
    let n_z = axis[2][0];

    R[0][0] = C_ + (n_x * n_x * (1 - C_));
    R[0][1] = (n_x * n_y * (1 - C_)) - (n_z * S_);
    R[0][2] = (n_x * n_z * (1 - C_)) + (n_y * S_);

    R[1][0] = (n_y * n_x * (1 - C_)) + (n_z * S_);
    R[1][1] = C_ + (n_y * n_y * (1 - C_));
    R[1][2] = (n_y * n_z * (1 - C_)) - (n_x * S_);

    R[2][0] = (n_z * n_x * (1 - C_)) - (n_y * S_);
    R[2][1] = (n_z * n_y * (1 - C_)) + (n_x * S_);
    R[2][2] = C_ + (n_z * n_z * (1 - C_));
    return R;
}

//行列の積
function Matrix_multi(data1,data2){
    let matrix_data1 = math.matrix(data1);
    let matrix_data2 = math.matrix(data2);
    let new_data = math.multiply(matrix_data1,matrix_data2);
    new_data = new_data.valueOf();
    return new_data;
}

//行列の和
function Matrix_add(data1,data2){
    //console.log("Matrix_add");
    //console.log(data1);
    //console.log(data2);
    let matrix_data1 = math.matrix(data1);
    let matrix_data2 = math.matrix(data2);
    let new_data = math.add(matrix_data1,matrix_data2);
    new_data = new_data.valueOf();
    return new_data;
}

//順運動学を解く
function ForwardKinematics(link_data){
    let link_num = link_data.length;
    for(let i = 1; i < link_num; i++){
        link_data[i]["rot"] = Matrix_multi(link_data[i-1].rot,Rodrigues(link_data[i].axis,link_data[i].angle));
        link_data[i]["pos"] = Matrix_add(Matrix_multi(link_data[i].rot,link_data[i].link_length),link_data[i-1].pos);
        
    }
}

//ヤコビアンの計算
function CalcJacobian(link_data){
  let target = math.matrix(link_data[link_data.length-1].pos);
  let J = math.zeros(6,link_data.length-1);
 // console.log("start jaco")
  for(let i = 1; i < link_data.length; i++){
    //console.log(i);
    //console.log(link_data[i].link_name);
    let link_rot = math.matrix(link_data[i].rot); 
    let link_axis = math.matrix(link_data[i].axis);
    let a = math.multiply(link_rot, link_axis);
    let link_pos = math.matrix(link_data[i].pos);
    let pos_diff = math.subtract(target,link_pos);
    let cross_data = math.cross(a , pos_diff);
    //console.log(JSON.stringify(link_rot.valueOf()));
    //console.log(JSON.stringify(link_axis.valueOf()));
    J = math.subset(J, math.index([0,1,2],i-1),cross_data.valueOf()[0]);
    J = math.subset(J, math.index([3,4,5],i-1),a);
  }
  return J.valueOf();
}

function pinv(jacobi){
  let jacobi_t = math.transpose(jacobi);
  //p_jacobi = math.multiply(jacobi_t,math.inv((math.multiply(jacobi,jacobi_t))));
  let p_jacobi = math.multiply(math.inv(math.multiply(jacobi_t,jacobi)),jacobi_t);
  //console.log(JSON.stringify(p_jacobi.valueOf()));
  return p_jacobi
  
}

function Rpy2rot(roll,pitch,yaw){
  //ワールド座標系に対しての回転
  let roll_rad = roll * Math.PI / 180;
  let pitch_rad = pitch * Math.PI / 180;
  let yaw_rad = yaw * Math.PI / 180;
  let roll_cos = math.cos(roll_rad);
  let roll_sin = math.sin(roll_rad);
  let pitch_cos = math.cos(pitch_rad);
  let pitch_sin = math.sin(pitch_rad)
  let yaw_cos = math.cos(yaw_rad);
  let yaw_sin = math.sin(yaw_rad);
  let R_r = math.matrix([[1,0,0],[0,roll_cos,-1*roll_sin],[0,roll_sin,roll_cos]]);
  let R_p = math.matrix([[pitch_cos,0,pitch_sin],[0,1,0],[-1*pitch_sin,0,pitch_cos]]);
  let R_y = math.matrix([[yaw_cos,-1*yaw_sin,0],[yaw_sin,yaw_cos,0],[0,0,1]]);
  return math.multiply(math.multiply(R_y,R_p),R_r).valueOf();
}

function Rot2omega(R){
  let alpha = (math.trace(R)-1)/2;
  if(math.abs(alpha - 1) < Number.EPSILON){
    //console.log("set 000")
    return math.matrix([[0],[0],[0]]);
  }else{
    let th = math.acos(alpha);
    
    let w = math.matrix([[R.valueOf()[2][1]-R.valueOf()[1][2]],[R.valueOf()[0][2]-R.valueOf()[2][0]],[R.valueOf()[1][0]-R.valueOf()[0][1]]])
    let test = 0.5*th/(math.sin(th));
    w = math.multiply(w,0.5*th/math.sin(th));
    //console.log(JSON.stringify(w));
    return w;
  }
}
//誤差の計算
function CalcVWerr(Target,Now_Link){//
  let Target_pos = math.matrix(Target.pos);
  let Now_Link_pos = math.matrix(Now_Link.pos);
  let Target_rot = math.matrix(Target.rot);
  let Now_Link_rot = math.matrix(Now_Link.rot);
  let perr = math.subtract(Target.pos, Now_Link.pos);
  let Rerr = math.multiply(math.inv(Now_Link_rot),Target_rot);
  //console.log(JSON.stringify(Rot2omega(Rerr).valueOf()));
  let werr = math.multiply(Now_Link_rot, Rot2omega(Rerr));
  let Error = math.zeros(6,1);
  Error = math.subset(Error,math.index([0,1,2],0),perr.valueOf());
  Error = math.subset(Error,math.index([3,4,5],0),werr.valueOf());
  //return math.matrix([perr.valueOf(),werr.valueOf()]);
  return Error;
}

function Show_arm(){
 console.log("########"); 
 //window.addEventListener('load', init);
 init()
 //document.addEventListener('load', init);
 function init() {
  console.log("start")
  let path_data;        
  //サイズを指定
  const width = 900;
  const height = 600;
        
  // レンダラーを作成
  const renderer = new THREE.WebGLRenderer({
      canvas: document.querySelector('#myCanvas')
  });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(width, height);
        
  // シーンを作成
  let scene = new THREE.Scene();
  //背景色を白に
  scene.background = new THREE.Color( 0xffffff);
  // カメラを作成
  const camera = new THREE.PerspectiveCamera(45, width / height);
  //camera.position.set(0, 0, +1000);
  camera.position.set(0, 0, +10);
  
  // カメラコントローラを作成
  const controls = new THREE.OrbitControls(camera);
  // 滑らかにカメラコントローラを制御する
  
    //座標軸を表示

  
  //gridを表示
  //縦、横のグリッドの数 : size/(size/step)
  //1グリッドのサイズ : size/step = voxelsize
  //今回の場合 : グリッド数 1000/(100/1000)=100個
  
  
  //let material = new THREE.MeshBasicMaterial( {color: 0xFF0000} );
  //let geometry = new THREE.BoxGeometry(40,40,20);//大きさを10倍に      
  tick();      
  // 毎フレーム時に実行されるループイベントです
  function tick() {
    if(flag == 0){
        console.log(scene.children);
        scene = new THREE.Scene();
        scene.background = new THREE.Color( 0xffffff);
        //for(let i=0; i < scene.children.length; i++){
        //    scene.remove(scene.children[i]);
        //    //scene.children[i].material.dispose();
        //    //scene.children[i].geometry.dispose();
        //    //scene.remove(scene.children[i]);
        //}
        console.log("clean")
        console.log(scene.children);
        let axis = new THREE.AxesHelper(1000);
        scene.add(axis);
        let joint_material = new THREE.MeshBasicMaterial( { color: 0xeeee00 } );
        let joint_geometry = new THREE.CircleGeometry( 0.1, 500);
        let geometry = new THREE.Geometry();
        let material = new THREE.LineBasicMaterial( { color:0x990000})
        geometry.vertices.push(new THREE.Vector3(0,0,0));
        let line = new THREE.Line(geometry, material);
        scene.add( line );
        for(let i=0; i < Link_list.length; i++){
            geometry.vertices.push(new THREE.Vector3(Link_list[i].pos[0][0],Link_list[i].pos[1][0],Link_list[i].pos[2][0])); 
            let Joint = new THREE.Mesh(joint_geometry, joint_material);
            if(i != Link_list.length-1){
            Joint.position.set(Link_list[i].pos[0][0],Link_list[i].pos[1][0],Link_list[i].pos[2][0])
            scene.add( Joint );
            }
            let line = new THREE.Line(geometry, material);
            scene.add( line );
        }
        flag = 1;
    }
      renderer.render(scene, camera); // レンダリング  
      requestAnimationFrame(tick);
  }
  }
}