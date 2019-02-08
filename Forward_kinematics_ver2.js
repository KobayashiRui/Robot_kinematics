var Link_list = [];
var name2link = {};
var link_num = 5;
var flag = 0;

Make_world();
var buf_link_name = "world_link";
for(let i=0; i < link_num; i++){
    let link_name = "link_"+i;
    let link_length = [[0],[1],[0]];
    let axis = [[0],[0],[1]];
    Make_link(link_name,buf_link_name,link_length,axis,20);
    name2link[link_name] = Link_list[i+1];
    Make_Controler(link_name);
    buf_link_name = link_name;
}

//console.log(Link_list);
ForwardKinematics(Link_list);
//console.log(Link_list);
//console.log(name2link);
console.log("hhhhhh")
Show_arm();


function Make_world(){
    let World_link = {link_name:"world_link",pos:[[0],[0],[0]],rot:[[1,0,0],[0,1,0],[0,0,1]]};
    Link_list.push(World_link);   
}
function Make_link(name,mather_name,link_length,axis,angle){
    let link = {link_name:name,mather_name:mather_name,link_length:link_length,axis:axis,angle:angle};
    Link_list.push(link);
}

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

function Matrix_multi(data1,data2){
    if(data1[0].length != data2.length){
        console.log("Error");
    }
    console.log("Datas")
    console.log(data1);
    console.log(data2);
    let num = data1[0].length;
    let row = data1.length;
    let line = data2[0].length;
    let new_data = [...Array(row)].map(() => Array(line).fill(0)); //行 列
    for(let i = 0; i < row; i++){
        for(let j =0; j < line; j++){
            let buf_data = 0;
            for(let k = 0; k < num; k++){
                buf_data += (data1[i][k] * data2[k][j])                
            }
            new_data[i][j] = buf_data;
        }
    }
    return new_data;
}

function Matrix_add(data1,data2){
    console.log("Matrix_add");
    console.log(data1);
    console.log(data2);
    let row = data1.length;
    let line = data1[0].length;
    let new_data = [...Array(row)].map(() => Array(line).fill(0)); //行 列
    for(let i = 0; i < row; i++){
        for(let j =0; j < line; j++){
            new_data[i][j] = data1[i][j] + data2[i][j];
        }
    }
    return new_data;
}

function ForwardKinematics(link_data){
    let link_num = link_data.length;
    for(let i = 1; i < link_num; i++){
        
        link_data[i]["pos"] = Matrix_add(Matrix_multi(Rodrigues(link_data[i].axis,link_data[i].angle),link_data[i].link_length),link_data[i-1].pos);
        link_data[i]["rot"] = Matrix_multi(link_data[i-1].rot,Rodrigues(link_data[i].axis,link_data[i].angle));
    }

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
  //const controls = new THREE.OrbitControls(camera);
  // 滑らかにカメラコントローラを制御する
  
    //座標軸を表示
  //let axis = new THREE.AxesHelper(1000);
  //scene.add(axis);
  
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
        let joint_material = new THREE.MeshBasicMaterial( { color: 0xeeee00 } );
        let joint_geometry = new THREE.CircleGeometry( 0.1, 500);
        let geometry = new THREE.Geometry();
        let material = new THREE.LineBasicMaterial( { color:0x990000})
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

function Controller(obj,name){
    let angle_data = parseFloat(obj.value);
    console.log(name + ":" + angle_data);
    name2link[name].angle = angle_data;
    ForwardKinematics(Link_list);
    flag = 0;
    
}

function Make_Controler(name){
    let elem = document.createElement('input');
    let base = document.getElementById("contro");
    let string_data = "Controller(this,'"+ name + "')"
    elem.setAttribute("type","range");
    elem.setAttribute("min","-180");
    elem.setAttribute("max","180");
    elem.setAttribute("value","0");
    elem.setAttribute("onInput",string_data);
    base.appendChild(elem);
}
