# Yet Another Robotics Toolbox (YART)

This `yet another` robotics toolbox written in MATLAB focuses on basic kinematic simulation of the kinematic structures of a rigid body. It support parsing the kinematic chain from a universal robot description format (URDF) file. 

Following is an exampled of a `chain` structure parsed from Sawyer:
```
                 name: 'sawyer'
              n_joint: 20
          n_rev_joint: 8
          joint_names: {1×20 cell}
      rev_joint_names: {'right_j0'  'head_pan'  'right_j1'  'right_j2'  'right_j3'  'right_j4'  'right_j5'  'right_j6'}
    parent_link_names: {1×20 cell}
     child_link_names: {1×20 cell}
                joint: [1×20 struct]
               n_link: 21
           link_names: {1×21 cell}
                 link: [1×21 struct]
              xyz_min: [3×1 double]
              xyz_max: [3×1 double]
              xyz_len: [3×1 double]
       rev_joint_idxs: [8×1 double]
                   dt: 0.0500
```
Two important sub-structures, `joint` and `link`, look like
```
>> chain.joint

ans = 

  1×20 struct array with fields:
  
    name
    parent
    childs
    p
    R
    q
    q_prev
    q_diff
    type
    a
    limit
    p_offset
    R_offset
    parent_link
    child_link
    v
    w
```
and
```
>> chain.link

ans = 

  1×21 struct array with fields:

    name
    joint_idx
    p_offset
    R_offset
    scale
    mesh_path
    fv
    bcube
    box
    box_scale
```

It also suppeorts an `augmentaed Jacobian method`for handling multiple IK targets with a straightforward syntax:
```
% Initial IK
ik = init_ik(chain);
% Add IK Targets 
ik = add_ik(ik,'joint_name','RWrj2',...
    'p',p_trgt1,'R',R_trgt1,'IK_P',1,'IK_R',1);
ik = add_ik(ik,'joint_name','LWrj2',...
    'p',p_trgt2,'R',R_trgt2,'IK_P',1,'IK_R',1);
while true
  ...
  % Run IK in Loop 
  [ik,chain,q] = onestep_ik(ik,chain,q);
  ...
end
```
It supports joint position limit handling by excluding the next possible violating joints. We tried soft-barrier-based methods but this simple heuristics work bettern in practice, at least in our cases. Handling self-collision will soon be implemented. 

### Interactive inverse kinematics

[![](http://img.youtube.com/vi/zHNi532F1-c/0.jpg)](http://www.youtube.com/watch?v=zHNi532F1-c "")

### Cmputing linear and angular velocities of links

[![](http://img.youtube.com/vi/klvGY-akl58/0.jpg)](http://www.youtube.com/watch?v=klvGY-akl58 "")

### TODO List:
- Handling motion capture data in `.bvh` formats. 
- Some basic machine learning methods such as Gaussian Processes and Bayesian Optimization.
- Mobile robotics and path planning methods such as rapidly exploring random trees (RRT) and vector field histogram (VFH).
- Model predictive control (MPC) 
- Trajectory optimization 

More information can be found in the following link:
https://sites.google.com/view/sungjoon-choi/yart.

Contact: Sungjoon Choi (sungjoon.s.choi@gmail.com)
