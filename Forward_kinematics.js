var Link_list = [];
var name2link = {};
var link_num = 2;

Make_world();
var buf_link_name = "world_link";
for(let i=0; i < link_num; i++){
    let link_name = "link_"+i;
    let link_length = [[0],[1],[0]];
    let axis = [[0],[0],[1]];
    Make_link(link_name,buf_link_name,link_length,axis,0);
    name2link[link_name] = Link_list[i+1];
    buf_link_name = link_name;
}
console.log(Link_list);
ForwardKinematics(Link_list);
console.log(Link_list);
console.log(name2link);

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
        
        link_data[i]["pos"] = Matrix_add(Matrix_multi(link_data[i-1].rot,link_data[i].link_length),link_data[i-1].pos);
        link_data[i]["rot"] = Matrix_multi(link_data[i-1].rot,Rodrigues(link_data[i].axis,link_data[i].angle));
    }

}