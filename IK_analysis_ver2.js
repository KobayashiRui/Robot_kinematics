class Dof6robotarm{
    //linkLength:[l0,l1,l1_2,l2,l3,l4,l5,l6]
    //linkLimit:[[s1_min,s1_max],[s2_min,s2_max], ....[s6_min,s6_max]]
    constructor(linkLength,nowAngle=[0,0,0,0,0,0],linkLimit=0,advanse = false){
        //todo 配列でない値が入った場合の例外処理
        if(advanse == true){
            if(linkLength.length < 8 ){
                let err = new Error();
                err.message = "リンクの長さが全て定義されていません";
                throw err;
            }
            if(homeAngel.length < 6 ){
                let err = new Error();
                err.message = "初期角度が定義されていません";
                throw err;
            }
            if(linkLimit.length < 6){
                let err = new Error();
                err.message = "ジョイントの角度制限が全て定義されていません";
                throw err;
            }
            for(let i=0; i < linkLimit.length; i++){
                if(linkLimit[i].length < 2){
                    let err = new Error();
                    err.message = "ジョイントの角度の最低値と最大値が全て定義されていません";
                    throw err;
                }
            }
            
            for(let i=0; i < linkLimit.length; i++){
                for(let j=0; j < 2; j++){
                   linkLimit[i][j] = linkLimit[i][j]*Math.PI/180; //degree to rad 
                }
            }
            this.linkLength = linkLength;
            this.linkLimit = linkLimit;
            this.nowAngel = nowAngle;
            this.advanse = true;
        }else{
            this.linkLength = linkLength;
            this.nowAngle = nowAngle;
            this.advanse = false;
            console.log(this.linkLength)
        }
    }
    //targetPst:[x,y,z]
    //targetRPY : [roll,pitch,yaw]
    ikSolve(targetPos,targetRPY){
        console.log("datas");
        console.log(targetPos);
        console.log(targetRPY);
        let rollrad  = targetRPY[0]*Math.PI/180;
        let pitchrad = targetRPY[1]*Math.PI/180;
        let yawrad   = targetRPY[2]*Math.PI/180; 
        let sin_r = Math.sin(rollrad);
        let cos_r = Math.cos(rollrad);
        let sin_p = Math.sin(pitchrad);
        let cos_p = Math.cos(pitchrad);
        let sin_y = Math.sin(yawrad);
        let cos_y = Math.cos(yawrad);
        
        let ax = (sin_p * cos_r * cos_y) + (sin_r * sin_y);
        let ay = (sin_p * cos_r * cos_y) - (sin_r * cos_y);
        let az = cos_p * cos_r;
        let bx = cos_p * cos_y;
        let by = cos_p * sin_y;
        let bz = -1 * sin_p;
        let p5x = targetPos[0] - ((this.linkLength[6] + this.linkLength[7])*ax);
        let p5y = targetPos[1] - ((this.linkLength[6] + this.linkLength[7])*ay);
        let p5z = targetPos[2] - ((this.linkLength[6] + this.linkLength[7])*az + this.linkLength[0]);
        //todo + pi
        let angle_list = [];
        for(let i = 0; i < 8; i++){
            console.log("Number:"+i);
            let angle_1;
            if(i < 4){
                angle_1 = Math.atan2(p5y,p5x);
            }else{
                angle_1 = Math.atan2(p5y,p5x) + Math.PI;
            }
            console.log(angle_1);
            
            //角度の条件判定
            if(Number.isNaN(angle_1)){
                continue;
            }
            if(this.advanse == true){
                if(angle_1 < this.linkLimit[0][0] || this.linkLimit[0][1] < angle_1){
                    continue;
                }
            }
                        
            let sin_angle_1 = Math.sin(angle_1);
            let cos_angle_1 = Math.cos(angle_1);
            let c3_numerator = (Math.pow(p5x-cos_angle_1*this.linkLength[2],2) + 
                        Math.pow(p5y-sin_angle_1*this.linkLength[2],2) + 
                        Math.pow(p5z-this.linkLength[1],2) - Math.pow(this.linkLength[3],2) -
                        Math.pow(this.linkLength[4] + this.linkLength[5],2));
            let c3_denominator = 2 * this.linkLength[3] * (this.linkLength[4]+this.linkLength[5]);
            let c3 = c3_numerator/c3_denominator;
            console.log("c3");
            console.log(c3_numerator);
            console.log(c3_denominator);
            console.log(c3);
            console.log(-1*Math.sqrt(1-c3));
            //todo sqrt 1-c3 + or - version
            let angle_3;
            if(i % 2 == 0){
                angle_3 = Math.atan2(Math.sqrt(1-Math.pow(c3,2)),c3);
            }else{
                angle_3 = Math.atan2(-1*Math.sqrt(1-Math.pow(c3,2)),c3);
            }
            console.log("angle_3" + angle_3);
            
            if(Number.isNaN(angle_3)){
                console.log("angle3 is null");
                continue;
            }
            if(this.advanse == true){
                if(angle_3 < this.linkLimit[2][0] || this.linkLimit[2][1] < angle_3){
                    continue;
                }
            }
            
            let sin_angle_3 = Math.sin(angle_3);
            let cos_angle_3 = Math.cos(angle_3);
            let A = Math.sqrt(Math.pow(p5x-cos_angle_1*this.linkLength[2],2) + 
                    Math.pow(p5y-sin_angle_1*this.linkLength[2],2))
            let B = p5z-this.linkLength[1];
            let M = this.linkLength[3] + (this.linkLength[4]+this.linkLength[5])*cos_angle_3;
            let N = (this.linkLength[4]+this.linkLength[5])*sin_angle_3;
            let angle_2 = Math.atan2(M*A-N*B, N*A+M*B);
            console.log(angle_2)
            
            if(Number.isNaN(angle_2)){
                continue;
            }
            if(this.advanse == true){
                if(angle_2 < this.linkLimit[1][0] || this.linkLimit[1][1] < angle_2){
                    continue;
                }
            }            
            
            let sin_angle_2_3 = Math.sin(angle_2 + angle_3);
            let cos_angle_2_3 = Math.cos(angle_2 + angle_3);
            let ax_2 = cos_angle_2_3 * (cos_angle_1 * ax + sin_angle_1 * ay) - sin_angle_2_3 * az;
            let ay_2 = -1 * sin_angle_1 * ax + cos_angle_1 * ay;
            let az_2 = sin_angle_2_3 * (cos_angle_1 * ax + sin_angle_1 * ay) + cos_angle_2_3 * az;
            let bx_2 = cos_angle_2_3 * (cos_angle_1 * bx + sin_angle_1 * by) - sin_angle_2_3 * bz;
            let by_2 = -1 * sin_angle_1 * bx + cos_angle_1 * by;
            let bz_2 = sin_angle_2_3 * (cos_angle_1 * bx + sin_angle_1 * by) + cos_angle_2_3 * bz;
            //todo + pi version
            let angle_4;
            if(i == 2 || i == 3 || i == 6 || i == 7){
                angle_4 = Math.atan2(ay_2,ax_2);
            }else{
                angle_4 = Math.atan2(ay_2,ax_2) + Math.PI;
            }
            console.log(angle_4);
            
            if(Number.isNaN(angle_4)){
                continue;
            }
            if(this.advanse == true){
                if(angle_4 < this.linkLimit[3][0] || this.linkLimit[3][1] < angle_4){
                    continue;
                }
            }            
            
            let sin_angle_4 = Math.sin(angle_4);
            let cos_angle_4 = Math.cos(angle_4);
            let angle_5 = Math.atan2(cos_angle_4*ax_2 + sin_angle_4 * ay_2 , az_2);
            console.log(angle_5);
            
            if(Number.isNaN(angle_5)){
                continue;
            }
            if(this.advanse == true){
                if(angle_5 < this.linkLimit[4][0] || this.linkLimit[4][1] < angle_5){
                    continue;
                }
            }            
            
            let sin_angle_5 = Math.sin(angle_5);
            let angle_6 = Math.atan2(cos_angle_4*by_2 - sin_angle_4 * bx_2 , -1 * bz_2 / sin_angle_5);
            console.log(angle_6);
            
            if(Number.isNaN(angle_6)){
                continue;
            }
            if(this.advanse == true){
                if(angle_6 < this.linkLimit[5][0] || this.linkLimit[5][1] < angle_6){
                    continue;
                }
            }            
            
            angle_list.push([angle_1,angle_2,angle_3,angle_4,angle_5,angle_6])
        }
            return angle_list;//rad data //NaNデータなしジョイント範囲内に収まった角度データ群
    }
    fkSolve(targetAngles){
        
    }
    
    optimalAngle(angle_list){
        let Errors = [];
        for(let i=0; i < angle_list.length; i++){
            let error_buf = [];
            for(let j=0; j < 6; j++){
                error_buf.push(this.nowAngle[j] - angle_list[i][j]);
            }
            Errors.push(math.norm(error_buf));
        }
        let Min_Error = Errors[0];
        let Min_Error_flag = 0;
        for(let i=1; i < Errors.length; i++){
            if(Errors[i] < Min_Error){
                Min_Error = Errors[i];
                Min_Error_flag = i;
            }
        }
        return Min_Error_flag;
    }
    
    changeNowAngle(angle_data){
        
    }
}


function Show_arm(link_list){
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
            geometry.vertices.push(new THREE.Vector3(link_list[i].pos[0][0],link_list[i].pos[1][0],link_list[i].pos[2][0])); 
            let Joint = new THREE.Mesh(joint_geometry, joint_material);
            if(i != link_list.length-1){
            Joint.position.set(link_list[i].pos[0][0],link_list[i].pos[1][0],link_list[i].pos[2][0])
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

var link_list = [1,1,1,1,1,1,0.5,0.1]
var robotarm = new Dof6robotarm(linkLength=link_list);
var target_pos = [0.5,0.5,3];
var target_rpy = [0,0,0];
console.log("angle_data");
var angle_data = robotarm.ikSolve(target_pos,target_rpy);
console.log(JSON.stringify(angle_data));
var vest_angle = robotarm.optimalAngle(angle_data);
console.log(vest_angle)
