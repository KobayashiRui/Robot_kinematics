class Dof6robotarm{
    //linkLength:[l0,l1,l1_2,l2,l3,l4,l5,l6]
    //linkLimit:[[s1_min,s1_max],[s2_min,s2_max], ....[s6_min,s6_max]]
    constructor(linkLength,homeAngel=0,linkLimit=0,advanse = false){
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
            this.linkLength = linkLength;
            this.linkLimit = linkLimit;
            this.homeAngel = homeAngel;
        }else{
            this.linkLength = linkLength;
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
            if(Number.isNaN(angle_1)){
                continue;
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
            let sin_angle_4 = Math.sin(angle_4);
            let cos_angle_4 = Math.cos(angle_4);
            let angle_5 = Math.atan2(cos_angle_4*ax_2 + sin_angle_4 * ay_2 , az_2);
            console.log(angle_5);
            if(Number.isNaN(angle_5)){
                continue;
            }
            let sin_angle_5 = Math.sin(angle_5);
            let angle_6 = Math.atan2(cos_angle_4*by_2 - sin_angle_4 * bx_2 , -1 * bz_2 / sin_angle_5);
            console.log(angle_6);
            if(Number.isNaN(angle_6)){
                continue;
            }
            angle_list.push([angle_1,angle_2,angle_3,angle_4,angle_5,angle_6])
        }
            return angle_list;//rad data
    }
}
var link_list = [1,1,1,1,1,1,0.5,0.1]
var robotarm = new Dof6robotarm(linkLength=link_list);
var target_pos = [0.5,0.5,3];
var target_rpy = [0,0,0];
console.log("angle_data");
var angle_data = robotarm.ikSolve(target_pos,target_rpy);
console.log(angle_data);