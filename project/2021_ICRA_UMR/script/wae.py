import os
import random
import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt
import numpy.matlib as npm # for sampler 
import scipy.io as sio
from inspect import isfunction
from util import get_nce_loss

class HyperSphereInsideSamplerClass(object):
    def __init__(self,name='hyper_sphere_inside_sampler',r=1,z_dim=16):
        self.name = name
        self.r = r
        self.z_dim = z_dim

    def sampler(self,n):
        z = np.random.randn(n,self.z_dim)
        z_norm = np.linalg.norm(z,axis=1)
        z_unit = z / npm.repmat(z_norm.reshape((-1,1)),1,self.z_dim) # on the surface of a hypersphere
        u = np.power(np.random.rand(n,1),(1/self.z_dim)*np.ones(shape=(n,1)))
        z_sphere = self.r * z_unit * npm.repmat(u,1,self.z_dim) # samples inside the hypersphere
        samples = z_sphere 
        return samples

    def plot(self,n=1000,tfs=20):
        samples = self.sampler(n=n)
        plt.figure(figsize=(6,6))
        plt.plot(samples[:,0],samples[:,1],'k.')
        plt.xlim(-self.r,self.r)
        plt.ylim(-self.r,self.r)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.title(self.name,fontsize=tfs)
        plt.show()    
        
class WassersteinAutoEncoderClass(object):
    """
    WAE implementation
    """
    def __init__(self,name='wae',xdim=784,zdim=16,
                 hdims_Q=[256]*2,hdims_P=[256]*2,hdims_D=[256]*2,
                 actv_Q=tf.nn.relu,actv_P=tf.nn.relu,actv_D=tf.nn.relu,
                 actv_latent=None,actv_out=None,
                 l2_coef=1.0,l1_coef=0.0,wd_coef=1e-6,beta=1.0):
        self.name = name
        self.xdim = xdim
        self.zdim = zdim
        self.hdims_Q = hdims_Q
        self.hdims_P = hdims_P
        self.hdims_D = hdims_D
        self.actv_Q = actv_Q
        self.actv_P = actv_P
        self.actv_D = actv_D
        self.actv_latent = actv_latent
        self.actv_out = actv_out
        
        self.l1_coef = l1_coef
        self.l2_coef = l2_coef
        self.wd_coef = wd_coef
        self.beta = beta
        
        self.z_radius = 1.0
        self.S = HyperSphereInsideSamplerClass(r=self.z_radius,z_dim=self.zdim)
        
        with tf.variable_scope(self.name,reuse=False):
            self.build_graph()
            
    def build_graph(self):
        kernel_init = tf.truncated_normal_initializer(mean=0.0,stddev=0.2)
        bias_init = tf.truncated_normal_initializer(mean=0.0,stddev=0.2)
        
        self.ph_x_real = tf.placeholder(shape=[None,self.xdim],dtype=tf.float32,name='x_real') # [n x x_dim]
        self.ph_x_real2 = tf.placeholder(shape=[None,self.xdim],dtype=tf.float32,name='x_real2') # [n x x_dim]
        self.ph_x_trgt = tf.placeholder(shape=[None,self.xdim],dtype=tf.float32,name='x_trgt') # [n x x_dim]
        self.ph_z_sample = tf.placeholder(shape=[None,self.zdim],dtype=tf.float32,name='z_sample') # [n x z_dim]
        self.lr = tf.placeholder(shape=[],dtype=tf.float32,name='lr') # [1]
        self.n = tf.shape(self.ph_x_real)[0] # number of batch
        
        # Encoder netowrk Q(z|x): ph_x_real => z_real
        with tf.variable_scope('Q',reuse=False):
            net = self.ph_x_real
            for h_idx,hdim in enumerate(self.hdims_Q):
                net = tf.layers.dense(net,hdim,activation=self.actv_Q,
                                      kernel_initializer=kernel_init,bias_initializer=bias_init,
                                      name='hid_Q_lin_%d'%(h_idx))
            self.z_real = tf.layers.dense(net,self.zdim,activation=self.actv_latent,
                                          kernel_initializer=kernel_init,bias_initializer=bias_init,
                                          name='z_real') # [n x z_dim]
            
        # (Duplicated for SSL) Encoder netowrk Q(z|x): ph_x_real2 => z_real2
        with tf.variable_scope('Q',reuse=True):
            net = self.ph_x_real2
            for h_idx,hdim in enumerate(self.hdims_Q):
                net = tf.layers.dense(net,hdim,activation=self.actv_Q,
                                      kernel_initializer=kernel_init,bias_initializer=bias_init,
                                      name='hid_Q_lin_%d'%(h_idx))
            self.z_real2 = tf.layers.dense(net,self.zdim,activation=self.actv_latent,
                                          kernel_initializer=kernel_init,bias_initializer=bias_init,
                                          name='z_real') # [n x z_dim]
            
        # Decoder netowrk P(x|z): ph_z_sample => x_sample
        with tf.variable_scope('P',reuse=False):
            net = self.ph_z_sample
            for h_idx,hdim in enumerate(self.hdims_P):
                net = tf.layers.dense(net,hdim,activation=self.actv_P,
                                      kernel_initializer=kernel_init,bias_initializer=bias_init,
                                      name='hid_P_lin_%d'%(h_idx))
            # Recon
            self.x_sample = tf.layers.dense(net,self.xdim,activation=self.actv_out,
                                           kernel_initializer=kernel_init,bias_initializer=bias_init,
                                           name='x_recon') # [n x x_dim]
            
        # Decoder netowrk P(x|z): z_real => x_recon
        with tf.variable_scope('P',reuse=True):
            net = self.z_real
            for h_idx,hdim in enumerate(self.hdims_P):
                net = tf.layers.dense(net,hdim,activation=self.actv_P,
                                      kernel_initializer=kernel_init,bias_initializer=bias_init,
                                      name='hid_P_lin_%d'%(h_idx))
            # Recon
            self.x_recon = tf.layers.dense(net,self.xdim,activation=self.actv_out,
                                            kernel_initializer=kernel_init,bias_initializer=bias_init,
                                            name='x_recon') # [n x x_dim]
        
        # Discriminator D(z): z_real => d_real
        with tf.variable_scope('D',reuse=False):
            net = self.z_real
            for h_idx,hdim in enumerate(self.hdims_D):
                net = tf.layers.dense(net,hdim,activation=self.actv_D,
                                      kernel_initializer=kernel_init,bias_initializer=bias_init,
                                      name='hid_D_lin_%d'%(h_idx))
            self.d_real_logits = tf.layers.dense(net,1,activation=None,
                                                 kernel_initializer=kernel_init,bias_initializer=bias_init,
                                                 name='d_logits') # [n x 1]
            self.d_real = tf.sigmoid(self.d_real_logits,name='d') # [n x 1]
        
        # (Reusing) Discriminator D(z): z_sample => d_fake
        with tf.variable_scope('D',reuse=True):
            net = self.ph_z_sample
            for h_idx,hdim in enumerate(self.hdims_D):
                net = tf.layers.dense(net,hdim,activation=self.actv_D,
                                      kernel_initializer=kernel_init,bias_initializer=bias_init,
                                      name='hid_D_lin_%d'%(h_idx))
            self.d_fake_logits = tf.layers.dense(net,1,activation=None,
                                                 kernel_initializer=kernel_init,bias_initializer=bias_init,
                                                 name='d_logits') # [n x 1]
            self.d_fake = tf.sigmoid(self.d_fake_logits,name='d') # [n x 1]
        
        # Loss 
        tfscewl = tf.nn.sigmoid_cross_entropy_with_logits
        self.d_loss_reals = tfscewl(logits=self.d_real_logits,labels=tf.zeros_like(self.d_real_logits)) # [n x 1]
        self.d_loss_fakes = tfscewl(logits=self.d_fake_logits,labels=tf.ones_like(self.d_fake_logits)) # [n x 1]
        self.d_losses = 0.5*self.d_loss_reals + 0.5*self.d_loss_fakes # [n x 1] -> adversarial loss? 
        self.g_losses = tfscewl(logits=self.d_real_logits,labels=tf.ones_like(self.d_real_logits)) # [n x 1]
        
        self.d_losses = self.beta*self.d_losses # [n x 1]
        self.g_losses = self.beta*self.g_losses # [n x 1]

        self.l1_losses = tf.reduce_sum(tf.abs(self.x_recon-self.ph_x_trgt),axis=1) # [n x 1] L1 norm
        self.l2_losses = tf.reduce_sum((self.x_recon-self.ph_x_trgt)**2,axis=1) # [n x 1] L2 norm
        self.recon_losses = self.l1_coef*self.l1_losses + self.l2_coef*self.l2_losses # [n x 1] reconstruction loss

        self.d_loss = tf.reduce_mean(self.d_losses) # [1]
        self.g_loss = tf.reduce_mean(self.g_losses) # [1]
        self.recon_loss = tf.reduce_mean(self.recon_losses) # [1]
        
        self.t_vars = [var for var in tf.trainable_variables() if '%s/'%(self.name) in var.name] 
        self.q_vars = [var for var in self.t_vars if '%s/Q'%(self.name) in var.name] # encoder 
        self.p_vars = [var for var in self.t_vars if '%s/P'%(self.name) in var.name] # encoder 
        self.d_vars = [var for var in self.t_vars if '%s/D'%(self.name) in var.name] # discriminator
        self.ae_vars = self.q_vars + self.p_vars # both encoder and decoder
        
        self.lr_recon_weight = tf.placeholder(shape=[],dtype=tf.float32,name='lr_recon_weight') 
        self.lr_d_weight = tf.placeholder(shape=[],dtype=tf.float32,name='lr_d_weight') 
        self.lr_g_weight = tf.placeholder(shape=[],dtype=tf.float32,name='lr_g_weight') 
        
        self.wd_loss = self.wd_coef*tf.reduce_sum(tf.stack([tf.nn.l2_loss(v) for v in self.ae_vars])) # [1]
        
        beta1,beta2,epsilon = 0.5,0.9,0.1 # 0.5,0.9
        self.optm_recon = tf.train.AdamOptimizer(
            self.lr*self.lr_recon_weight,beta1=beta1,beta2=beta2,epsilon=epsilon).minimize(
            self.recon_loss+self.wd_loss,var_list=self.ae_vars)
        self.optm_d = tf.train.AdamOptimizer(
            self.lr*self.lr_d_weight,beta1=beta1,beta2=beta2,epsilon=epsilon).minimize(
            self.d_loss,var_list=self.d_vars)
        self.optm_g = tf.train.AdamOptimizer(
            self.lr*self.lr_g_weight,beta1=beta1,beta2=beta2,epsilon=epsilon).minimize(
            self.g_loss,var_list=self.q_vars)
        
    # Update network parameters
    def update(self,sess,x_batch4recon,x_batch4latent,
               n_g_update=1,lr=0.001,
               lr_recon_weight=1.0,lr_d_weight=0.1,lr_g_weight=0.5):
        """
            Update WAE
        """
        
        # Update discriminator and generator for latent prior 
        batch_size = x_batch4latent.shape[0]
        z_sample = self.S.sampler(batch_size)
        feeds = {self.ph_x_real:x_batch4latent,
                 self.ph_z_sample:z_sample,self.lr:lr,
                 self.lr_d_weight:lr_d_weight,self.lr_g_weight:lr_g_weight}
        d_loss,_ = sess.run([self.d_loss,self.optm_d],feed_dict=feeds)
        feeds = {self.ph_x_real:x_batch4latent,
                 self.ph_z_sample:z_sample,self.lr:lr,
                 self.lr_d_weight:lr_d_weight,self.lr_g_weight:lr_g_weight}
        for _ in range(n_g_update): 
            g_loss,_ = sess.run([self.g_loss,self.optm_g],feed_dict=feeds)
        
        # Update both encoder and decoder (reconstruction loss)
        feeds = {self.ph_x_real:x_batch4recon,self.ph_x_trgt:x_batch4recon,
                 self.lr:lr,self.lr_recon_weight:lr_recon_weight}
        recon_loss,wd_loss,_ = sess.run([self.recon_loss,self.wd_loss,self.optm_recon],feed_dict=feeds)
        
        return recon_loss,d_loss,g_loss,wd_loss
    
    # Update network parameters
    def update2(self,sess,x_batch4recon_in,x_batch4recon_out,x_batch4latent,
               n_g_update=1,lr=0.001,
               lr_recon_weight=1.0,lr_d_weight=0.1,lr_g_weight=0.5):
        """
            Update WAE
        """
        
        # Update discriminator and generator for latent prior 
        batch_size = x_batch4latent.shape[0]
        z_sample = self.S.sampler(batch_size)
        feeds = {self.ph_x_real:x_batch4latent,
                 self.ph_z_sample:z_sample,self.lr:lr,
                 self.lr_d_weight:lr_d_weight,self.lr_g_weight:lr_g_weight}
        d_loss,_ = sess.run([self.d_loss,self.optm_d],feed_dict=feeds)
        feeds = {self.ph_x_real:x_batch4latent,
                 self.ph_z_sample:z_sample,self.lr:lr,
                 self.lr_d_weight:lr_d_weight,self.lr_g_weight:lr_g_weight}
        for _ in range(n_g_update): 
            g_loss,_ = sess.run([self.g_loss,self.optm_g],feed_dict=feeds)
        
        # Update both encoder and decoder (reconstruction loss)
        feeds = {self.ph_x_real:x_batch4recon_in,self.ph_x_trgt:x_batch4recon_out,
                 self.lr:lr,self.lr_recon_weight:lr_recon_weight}
        recon_loss,wd_loss,_ = sess.run([self.recon_loss,self.wd_loss,self.optm_recon],feed_dict=feeds)
        
        return recon_loss,d_loss,g_loss,wd_loss
    
    # Save network information to matfile 
    def save_to_mat(self,sess,epoch=0,suffix='',SAVE_MAT=True,CHANGE_VAR_NAME=True,VERBOSE=True):
        """
            Save to a mat file 
        """
        v_names,d = [],{}
        c_name = self.name
        t_vars = self.t_vars # trainable variables
        for v_idx,var in enumerate(t_vars):
            w_name,v_name = var.name,var.name
            v_shape = var.get_shape().as_list()
            if CHANGE_VAR_NAME:
                v_name = v_name.replace('/','_') # replace '/' => '_'
                v_name = v_name.replace(':','_') # replace ':' => '_'
                v_name = v_name.replace('%s_'%(c_name),'') # remove class name
                v_name = (v_name[::-1].split('_',1)[1])[::-1] 
                # remove characters after LAST '_' (hid_0_kernel_0 -> hid_0_kernel)
            if ('kernel:' in w_name) or ('bias:' in w_name) or \
                ('moving_mean:' in w_name) or ('moving_variance:' in w_name) or \
                ('gamma:' in w_name) or ('beta:' in w_name): 
                v_names.append(v_name)
                v_val = sess.run(var)
                d[v_name] = v_val
                
        # Class properties
        props = ['name','xdim','zdim','hdims_Q','hdims_P','hdims_D',
                 'actv_Q','actv_P','actv_D']
        for prop in props:
            if (isfunction(getattr(self,prop))): # function name
                d[prop] = getattr(self,prop).__name__
            else: # others
                d[prop] = getattr(self,prop)
                
        # Validation data
        props = ['x_real_vald','z_real_vald','x_recon_vald','epoch']
        n_vald = 10
        x_real_vald = np.random.randn(n_vald,self.xdim)
        z_real_vald = sess.run(self.z_real,feed_dict={self.ph_x_real:x_real_vald})
        x_recon_vald = sess.run(self.x_recon,feed_dict={self.ph_x_real:x_real_vald})
        for prop in props:
            d[prop] = vars()[prop]
            
        # Check names and types of things to save
        for k_idx,key in enumerate(d.keys()):
            item_type = type(d[key]).__name__
            if VERBOSE:
                print ("  [%02d] Name:[%s] Type:[%s]."%(k_idx,key,item_type))
        
        # Save to a mat file
        if SAVE_MAT:
            dir_path = 'nets/%s'%(self.name)
            mat_path = os.path.join(dir_path,'weights%s.mat'%(suffix))
            if not os.path.exists(dir_path):
                os.makedirs(dir_path)
                print ("[%s] created."%(dir_path))
            sio.savemat(mat_path,d) # save to a mat file
            print ("[%s] saved. Size is[%.3f]MB."%(mat_path,os.path.getsize(mat_path) / 1000000))
            
    # Save network information to matfile 
    def restore_from_mat(self,mat_path,sess,CHANGE_VAR_NAME=True,VERBOSE=True):
        """
            Restore from the mat file 
        """
        c_name = self.name
        t_vars = self.t_vars # trainable variables
        l = sio.loadmat(mat_path)
        d = {}
        for v_idx,var in enumerate(t_vars):
            w_name,v_name = var.name,var.name
            v_shape = var.get_shape().as_list()
            if CHANGE_VAR_NAME:
                v_name = v_name.replace('/','_') # replace '/' => '_'
                v_name = v_name.replace(':','_') # replace ':' => '_'
                v_name = v_name.replace('%s_'%(c_name),'') # remove class name
                v_name = (v_name[::-1].split('_',1)[1])[::-1] 
                # remove characters after LAST '_' (hid_0_kernel_0 -> hid_0_kernel)
            if ('kernel:' in w_name) or ('bias:' in w_name) or \
                ('moving_mean:' in w_name) or ('moving_variance:' in w_name) or \
                ('gamma:' in w_name) or ('beta:' in w_name): 
                # Assign loaded tensors to approprate places with appropriate shapes                
                val = l[v_name] # get loaded value
                newshape = sess.run(tf.shape(var)) # get the shape of original TF tensor
                sess.run(tf.assign(var,np.reshape(val,newshape=newshape))) # assign 
                if VERBOSE:
                    print ("[%d] name:[%s] restored from [%s]."%(v_idx,var.name,v_name))
        print ("Variables of [%s] restored from [%s]."%(self.name,mat_path))
                    

def plot_wae_mnist_2d_results(W,sess,x,y):
    n = x.shape[0]
    feed_dict = {W.ph_x_real:x}
    z_real,x_recon = sess.run([W.z_real,W.x_recon],feed_dict=feed_dict)
    
    # Plot z sample and z mu 
    plt.figure(figsize=(15,6))
    plt.subplot(1,2,1)
    z_prior = W.S.sampler(n)
    plt.scatter(z_prior[:,0],z_prior[:,1],c='k',cmap='jet')
    plt.title('Z Prior',fontsize=20);plt.grid()
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlim([-1.5,1.5]); plt.ylim([-1.5,1.5]); 
    plt.subplot(1,2,2)
    plt.scatter(z_real[:,0],z_real[:,1],c=y,cmap='jet')
    plt.title('Z Sample',fontsize=20);plt.colorbar();plt.grid()
    plt.xlim([-1.5,1.5]); plt.ylim([-1.5,1.5]); plt.show()
        
    # Plot input and reconstructed inputs
    n_sample = 5
    randperm_idxs = np.random.permutation(n)[:n_sample]
    fig = plt.figure(figsize=(15,3))
    for i in range(n_sample):
        plt.subplot(1,n_sample,i+1)
        plt.imshow(x[randperm_idxs[i],:].reshape(28,28),vmin=0,vmax=1,cmap="gray")
    fig.suptitle("Training Inputs",fontsize=20);plt.show()

    fig = plt.figure(figsize=(15,3))
    for i in range(n_sample):
        plt.subplot(1,n_sample,i+1)
        plt.imshow(x_recon[randperm_idxs[i],:].reshape(28,28),vmin=0,vmax=1,cmap="gray")
    fig.suptitle("Reconstructed Inputs",fontsize=20);plt.show()

    # Plot generated inputs
    z = W.S.sampler(n_sample)
    feed_dict = {W.ph_z_sample:z}
    x_gen = sess.run(W.x_sample,feed_dict=feed_dict)
    fig = plt.figure(figsize=(15,3))
    for i in range(n_sample):
        plt.subplot(1,n_sample,i+1)
        plt.imshow(x_gen[i,:].reshape(28,28),vmin=0,vmax=1,cmap="gray")
    fig.suptitle("Generated Inputs",fontsize=20);plt.show()

    # Plot the latent space
    nx = ny = 20
    xmin,xmax,ymin,ymax = -1,+1,-1,+1
    x_values,y_values = np.linspace(xmin,xmax,nx),np.linspace(ymin,ymax,ny)
    canvas = np.empty((28*ny, 28*nx))
    for i, yi in enumerate(x_values):
        for j, xi in enumerate(y_values):
            feed_dict = {W.ph_z_sample:np.array([[xi, yi]])}
            x_gen = sess.run(W.x_sample,feed_dict=feed_dict)
            canvas[(nx-i-1)*28:(nx-i)*28, j*28:(j+1)*28] = x_gen[0].reshape(28, 28)
    plt.figure(figsize=(8, 10))        
    Xi, Yi = np.meshgrid(x_values, y_values)
    plt.imshow(canvas, origin="upper", cmap="gray")
    plt.title("Latent Space",fontsize=20);plt.tight_layout();plt.show()

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

    
class WassersteinAutoEncoderClass_v2(object):
    def __init__(self,name='wae',xdim=2,zdim=2,
                 hdims_Q=[256]*2,hdims_P=[256]*2,hdims_D=[256]*2,
                 actv_Q=tf.nn.relu,actv_P=tf.nn.relu,actv_D=tf.nn.relu,
                 actv_latent=None,actv_out=None,
                 adam_beta1=0.5,adam_beta2=0.9,adam_epsilon=1e-1,
                 ki=tf.contrib.layers.variance_scaling_initializer() 
                ):
        self.name = name
        self.xdim = xdim
        self.zdim = zdim
        self.hdims_Q = hdims_Q
        self.hdims_P = hdims_P
        self.hdims_D = hdims_D
        self.actv_Q = actv_Q
        self.actv_P = actv_P
        self.actv_D = actv_D
        self.actv_latent = actv_latent
        self.actv_out = actv_out
        self.S = HyperSphereInsideSamplerClass(r=1.0,z_dim=self.zdim)
        
        self.adam_beta1 = adam_beta1
        self.adam_beta2 = adam_beta2
        self.adam_epsilon = adam_epsilon
        
        self.ki = ki
        
        with tf.variable_scope(self.name):
            self.build_model()
            self.build_graph()
        
    def build_model(self):
        """
            Build model
        """
        # ki = tf.contrib.layers.xavier_initializer()
        # ki = tf.contrib.layers.variance_scaling_initializer() 
        # ki = tf.truncated_normal_initializer(stddev=0.1)
        ki = self.ki
        bi = tf.constant_initializer(value=0)
        self.ph_x_real = tf.placeholder(shape=[None,self.xdim],dtype=tf.float32,name='ph_x_real') # [n x x_dim]
        self.ph_x_real_r = tf.placeholder(shape=[None,self.xdim],dtype=tf.float32,name='ph_x_real_r') # [n x x_dim]
        self.ph_x_trgt = tf.placeholder(shape=[None,self.xdim],dtype=tf.float32,name='ph_x_trgt') # [n x x_dim]
        self.ph_z_sample = tf.placeholder(shape=[None,self.zdim],dtype=tf.float32,name='ph_z_sample') # [n x z_dim]
        
        # Encoder netowrk Q(z|x): ph_x_real => z_real
        with tf.variable_scope('Q',reuse=False):
            net = self.ph_x_real
            for h_idx,hdim in enumerate(self.hdims_Q):
                net = tf.layers.dense(net,hdim,activation=self.actv_Q,
                                      kernel_initializer=ki,bias_initializer=bi,
                                      name='hid_Q_lin_%d'%(h_idx))
            self.z_real = tf.layers.dense(net,self.zdim,activation=self.actv_latent,
                                          kernel_initializer=ki,bias_initializer=bi,
                                          name='z_real') # [n x z_dim]
        
        # (Reused) Encoder netowrk Q(z|x): ph_x_real_r => z_real_r
        with tf.variable_scope('Q',reuse=True):
            net = self.ph_x_real_r
            for h_idx,hdim in enumerate(self.hdims_Q):
                net = tf.layers.dense(net,hdim,activation=self.actv_Q,
                                      kernel_initializer=ki,bias_initializer=bi,
                                      name='hid_Q_lin_%d'%(h_idx))
            self.z_real_r = tf.layers.dense(net,self.zdim,activation=self.actv_latent,
                                            kernel_initializer=ki,bias_initializer=bi,
                                            name='z_real') # [n x z_dim]
        
        # Decoder netowrk P(x|z): ph_z_sample => x_sample
        with tf.variable_scope('P',reuse=False):
            net = self.ph_z_sample
            for h_idx,hdim in enumerate(self.hdims_P):
                net = tf.layers.dense(net,hdim,activation=self.actv_P,
                                      kernel_initializer=ki,bias_initializer=bi,
                                      name='hid_P_lin_%d'%(h_idx))
            self.x_sample = tf.layers.dense(net,self.xdim,activation=self.actv_out,
                                           kernel_initializer=ki,bias_initializer=bi,
                                           name='x_recon') # [n x x_dim]
            
        # (Reused) Decoder netowrk P(x|z): z_real => x_recon
        with tf.variable_scope('P',reuse=True):
            net = self.z_real
            for h_idx,hdim in enumerate(self.hdims_P):
                net = tf.layers.dense(net,hdim,activation=self.actv_P,
                                      kernel_initializer=ki,bias_initializer=bi,
                                      name='hid_P_lin_%d'%(h_idx))
            self.x_recon = tf.layers.dense(net,self.xdim,activation=self.actv_out,
                                            kernel_initializer=ki,bias_initializer=bi,
                                            name='x_recon') # [n x x_dim]
        # Discriminator D(z): z_real => d_real
        with tf.variable_scope('D',reuse=False):
            net = self.z_real
            for h_idx,hdim in enumerate(self.hdims_D):
                net = tf.layers.dense(net,hdim,activation=self.actv_D,
                                      kernel_initializer=ki,bias_initializer=bi,
                                      name='hid_D_lin_%d'%(h_idx))
            self.d_real_logits = tf.layers.dense(net,1,activation=None,
                                                 kernel_initializer=ki,bias_initializer=bi,
                                                 name='d_logits') # [n x 1]
            self.d_real = tf.sigmoid(self.d_real_logits,name='d') # [n x 1]
        
        # (Reused) Discriminator D(z): z_sample => d_fake
        with tf.variable_scope('D',reuse=True):
            net = self.ph_z_sample
            for h_idx,hdim in enumerate(self.hdims_D):
                net = tf.layers.dense(net,hdim,activation=self.actv_D,
                                      kernel_initializer=ki,bias_initializer=bi,
                                      name='hid_D_lin_%d'%(h_idx))
            self.d_fake_logits = tf.layers.dense(net,1,activation=None,
                                                 kernel_initializer=ki,bias_initializer=bi,
                                                 name='d_logits') # [n x 1]
            self.d_fake = tf.sigmoid(self.d_fake_logits,name='d') # [n x 1]
            
        
    def build_graph(self):
        """
            Build computational graph
        """
        tfscewl = tf.nn.sigmoid_cross_entropy_with_logits
        
        # Latent prior loss
        self.beta = tf.placeholder(shape=[],dtype=tf.float32,name='beta') # [1]
        self.d_loss_reals = tfscewl(logits=self.d_real_logits,labels=tf.zeros_like(self.d_real_logits)) # [n x 1]
        self.d_loss_fakes = tfscewl(logits=self.d_fake_logits,labels=tf.ones_like(self.d_fake_logits)) # [n x 1]
        self.d_losses = 0.5*self.d_loss_reals + 0.5*self.d_loss_fakes # [n x 1] -> adversarial loss? 
        self.g_losses = tfscewl(logits=self.d_real_logits,labels=tf.ones_like(self.d_real_logits)) # [n x 1]
        self.d_losses = self.beta*self.d_losses # [n x 1]
        self.g_losses = self.beta*self.g_losses # [n x 1]
        self.d_loss = tf.reduce_mean(self.d_losses) # [1]
        self.g_loss = tf.reduce_mean(self.g_losses) # [1]
        
        # Reconstruction loss
        self.l1_coef = tf.placeholder(shape=[],dtype=tf.float32,name='l1_coef') # [1]
        self.l2_coef = tf.placeholder(shape=[],dtype=tf.float32,name='l2_coef') # [1]
        self.l1_recon = tf.reduce_sum(tf.abs(self.x_recon-self.ph_x_trgt),axis=1) # [n x 1] L1 norm
        self.l2_recon = tf.reduce_sum((self.x_recon-self.ph_x_trgt)**2,axis=1) # [n x 1] L2 norm
        self.recon_losses = self.l1_coef*self.l1_recon + self.l2_coef*self.l2_recon # [n x 1] reconstruction loss
        self.recon_loss = tf.reduce_mean(self.recon_losses) # [1]
        
        # Variables
        self.t_vars = [var for var in tf.trainable_variables() if '%s/'%(self.name) in var.name] 
        self.q_vars = [var for var in self.t_vars if '%s/Q'%(self.name) in var.name] # encoder 
        self.p_vars = [var for var in self.t_vars if '%s/P'%(self.name) in var.name] # decoder 
        self.d_vars = [var for var in self.t_vars if '%s/D'%(self.name) in var.name] # discriminator
        self.ae_vars = self.q_vars + self.p_vars # both encoder and decoder
        
        # Weight decay (for auto-encoder weights)
        self.wd_coef = tf.placeholder(shape=[],dtype=tf.float32,name='wd_coef') # [1]
        self.wd_loss = self.wd_coef*tf.reduce_sum(tf.stack([tf.nn.l2_loss(v) for v in self.ae_vars])) # [1]
        
        # Optimizer
        self.lr_recon = tf.placeholder(shape=[],dtype=tf.float32,name='lr_recon') # [1]
        with tf.variable_scope('%s/optm_recon'%(self.name),reuse=False):
            self.optm_recon = tf.train.AdamOptimizer(
                self.lr_recon,beta1=self.adam_beta1,beta2=self.adam_beta2,epsilon=self.adam_epsilon).minimize(
                self.recon_loss+self.wd_loss,var_list=self.ae_vars) 
        self.lr_d = tf.placeholder(shape=[],dtype=tf.float32,name='lr_d') # [1]
        with tf.variable_scope('%s/optm_d'%(self.name),reuse=False):
            self.optm_d = tf.train.AdamOptimizer(
                self.lr_d,beta1=self.adam_beta1,beta2=self.adam_beta2,epsilon=self.adam_epsilon).minimize(
                self.d_loss,var_list=self.d_vars)
        self.lr_g = tf.placeholder(shape=[],dtype=tf.float32,name='lr_g') # [1]
        with tf.variable_scope('%s/optm_g'%(self.name),reuse=False):
            self.optm_g = tf.train.AdamOptimizer(
                self.lr_g,beta1=self.adam_beta1,beta2=self.adam_beta2,epsilon=self.adam_epsilon).minimize(
                self.g_loss,var_list=self.q_vars)
        
    def update(self,sess,x_recon_in,x_recon_out,x_latent,
               beta=1.0,l1_coef=1.0,l2_coef=1.0,wd_coef=1e-6,
               lr_recon=1e-3,lr_d=1e-3,lr_g=1e-3
              ):
        """
            Update
        """
        # Reconstruction
        if (l1_coef > 0) or (l2_coef > 0):
            feeds = {self.ph_x_real:x_recon_in,self.ph_x_trgt:x_recon_out,
                     self.l1_coef:l1_coef,self.l2_coef:l2_coef,self.wd_coef:wd_coef,
                     self.lr_recon:lr_recon}
            recon_loss,wd_loss,_ = sess.run([self.recon_loss,self.wd_loss,self.optm_recon],feed_dict=feeds)
        else:
            recon_loss,wd_loss = 0.0,0.0
        
        # Latent prior D
        if (beta > 0):
            n_latent = x_latent.shape[0]
            z_sample = self.S.sampler(n_latent)
            feeds = {self.ph_x_real:x_latent,self.ph_z_sample:z_sample,
                     self.beta:beta,self.lr_d:lr_d}
            d_loss,_ = sess.run([self.d_loss,self.optm_d],feed_dict=feeds)

            # Latent prior G
            z_sample = self.S.sampler(n_latent)
            feeds = {self.ph_x_real:x_latent,
                     self.beta:beta,self.lr_g:lr_g}
            g_loss,_ = sess.run([self.g_loss,self.optm_g],feed_dict=feeds)
        else:
            d_loss = 0
            g_loss = 0
        
        return recon_loss,wd_loss,d_loss,g_loss
    
    def save_to_mat(self,sess,it=0,suffix='',SAVE_MAT=True,CHANGE_VAR_NAME=True,VERBOSE=True):
        """
            Save to a mat file 
        """
        v_names,d = [],{}
        c_name = self.name
        t_vars = self.t_vars # trainable variables
        for v_idx,var in enumerate(t_vars):
            w_name,v_name = var.name,var.name
            v_shape = var.get_shape().as_list()
            if CHANGE_VAR_NAME:
                v_name = v_name.replace('/','_') # replace '/' => '_'
                v_name = v_name.replace(':','_') # replace ':' => '_'
                v_name = v_name.replace('%s_'%(c_name),'') # remove class name
                v_name = (v_name[::-1].split('_',1)[1])[::-1] 
                # remove characters after LAST '_' (hid_0_kernel_0 -> hid_0_kernel)
            if ('kernel:' in w_name) or ('bias:' in w_name) or \
                ('moving_mean:' in w_name) or ('moving_variance:' in w_name) or \
                ('gamma:' in w_name) or ('beta:' in w_name): 
                v_names.append(v_name)
                v_val = sess.run(var)
                d[v_name] = v_val
                
        # Class properties
        props = ['name','xdim','zdim','hdims_Q','hdims_P','hdims_D',
                 'actv_Q','actv_P','actv_D']
        for prop in props:
            if (isfunction(getattr(self,prop))): # function name
                d[prop] = getattr(self,prop).__name__
            else: # others
                d[prop] = getattr(self,prop)
                
        # Validation data
        props = ['x_real_vald','z_real_vald','x_recon_vald','it']
        n_vald = 10
        x_real_vald = np.random.randn(n_vald,self.xdim)
        z_real_vald = sess.run(self.z_real,feed_dict={self.ph_x_real:x_real_vald})
        x_recon_vald = sess.run(self.x_recon,feed_dict={self.ph_x_real:x_real_vald})
        for prop in props:
            d[prop] = vars()[prop]
            
        # Check names and types of things to save
        for k_idx,key in enumerate(d.keys()):
            item_type = type(d[key]).__name__
            if VERBOSE:
                print ("  [%02d] Name:[%s] Type:[%s]."%(k_idx,key,item_type))
        
        # Save to a mat file
        if SAVE_MAT:
            dir_path = 'nets/%s'%(self.name)
            mat_path = os.path.join(dir_path,'weights%s.mat'%(suffix))
            if not os.path.exists(dir_path):
                os.makedirs(dir_path)
                print ("[%s] created."%(dir_path))
            sio.savemat(mat_path,d) # save to a mat file
            print ("[%s] saved. Size is[%.3f]MB."%(mat_path,os.path.getsize(mat_path) / 1000000))
            
    def restore_from_mat(self,mat_path,sess,CHANGE_VAR_NAME=True,VERBOSE=True):
        """
            Restore from the mat file 
        """
        c_name = self.name
        t_vars = self.t_vars # trainable variables
        l = sio.loadmat(mat_path)
        d = {}
        for v_idx,var in enumerate(t_vars):
            w_name,v_name = var.name,var.name
            v_shape = var.get_shape().as_list()
            if CHANGE_VAR_NAME:
                v_name = v_name.replace('/','_') # replace '/' => '_'
                v_name = v_name.replace(':','_') # replace ':' => '_'
                v_name = v_name.replace('%s_'%(c_name),'') # remove class name
                v_name = (v_name[::-1].split('_',1)[1])[::-1] 
                # remove characters after LAST '_' (hid_0_kernel_0 -> hid_0_kernel)
            if ('kernel:' in w_name) or ('bias:' in w_name) or \
                ('moving_mean:' in w_name) or ('moving_variance:' in w_name) or \
                ('gamma:' in w_name) or ('beta:' in w_name): 
                # Assign loaded tensors to approprate places with appropriate shapes                
                val = l[v_name] # get loaded value
                newshape = sess.run(tf.shape(var)) # get the shape of original TF tensor
                sess.run(tf.assign(var,np.reshape(val,newshape=newshape))) # assign 
                if VERBOSE:
                    print ("[%d] name:[%s] restored from [%s]."%(v_idx,var.name,v_name))
        print ("Variables of [%s] restored from [%s]."%(self.name,mat_path))
        
        
        
def run_wae_synthetic_experiment(USE_DPP=True,seed=1):
    
    from scipy.spatial import distance
    from util import get_2d_synthetic_data_for_swae,plot_2d_synthetic_data_for_swae,\
        gpu_sess,get_dpp_subset_from_K,get_dpp_subset_trim
    
    # Dataset
    MAKE_IMBALANCE = True # True / False
    x_relaxed,x_feasible,y_feasible,c = get_2d_synthetic_data_for_swae(
        MAKE_IMBALANCE=MAKE_IMBALANCE)
    n = x_relaxed.shape[0]
    np.random.seed(seed=seed);n_train = 100;idx_sel = np.random.permutation(n)[:n_train]
    x_train_rel,x_train_feas = x_relaxed[idx_sel,:],x_feasible[idx_sel,:]
    y_train_feas,c_train = y_feasible[idx_sel,:],c[idx_sel,:]
    plot_2d_synthetic_data_for_swae(x_relaxed,x_feasible,y_feasible,c,MAKE_IMBALANCE=MAKE_IMBALANCE,
        x_train_rel=x_train_rel,x_train_feas=x_train_feas,y_train_feas=y_train_feas)
    print ("# of total data:[%d] / # of train data:[%d]"%(n,n_train))
    
    # Mean-zero 
    x_subset,_ = get_dpp_subset_trim(x_feasible,1000,alpha=1000)
    x_mean = np.mean(x_subset,axis=0)
    x_feasible = x_feasible - x_mean

    # Hyperparameter
    max_iter,batch_size,print_every,plot_every = 10000,256,1000,2000
    hdims,actv_Q,actv_P,actv_D = [64]*3,tf.nn.relu,tf.nn.relu,tf.nn.relu
    beta = 1.0 # latent prior beta 
    l1_coef,l2_coef,wd_coef = 1.0,1.0,1e-6 # recon coef 
    lr_recon,lr_d,lr_g = 1e-3,1e-3,1e-3
    adam_beta1,adam_beta2,adam_epsilon=0.9,0.9,1e-0
    ki = tf.contrib.layers.variance_scaling_initializer() 
    # ki = tf.truncated_normal_initializer(stddev=0.1) 
    # ki = tf.contrib.layers.xavier_initializer()
    
    # WAE
    tf.reset_default_graph()
    W = WassersteinAutoEncoderClass_v2(
        name='wae',xdim=2,zdim=2,
        hdims_Q=hdims,hdims_P=hdims,hdims_D=hdims,
        actv_Q=actv_Q,actv_P=actv_P,actv_D=actv_D,
        actv_latent=None,actv_out=None,ki=ki)
    sess = gpu_sess() # open session, old legacy of TF 
    tf.set_random_seed(seed=seed); np.random.seed(seed=seed)
    sess.run(tf.global_variables_initializer())
    
    # DPP-sets
    np.random.seed(seed=seed)
    idx_total,K_dpp_xs,x_train_dpps,n_dpp = np.random.permutation(n),[],[],2000
    for i_idx in range(n//n_dpp):
        idx_dpp = idx_total[(i_idx)*n_dpp:(i_idx+1)*n_dpp]
        x_train_dpp = x_feasible[idx_dpp,:] # subset of x_relaxed x (or x_feasible)
        x_train_dpps.append(x_train_dpp)
        K_dpp_x = np.exp(-1000*distance.cdist(x_train_dpp,x_train_dpp,'sqeuclidean'))
        K_dpp_xs.append(K_dpp_x)
    print ("We have [%d] Ks."%(len(K_dpp_xs)))
    
    # Loop
    lr_d_weight = 1.0
    for it in range(max_iter): 

        # DPP subset
        idx = np.random.permutation(len(K_dpp_xs))[0]
        x_train_dpp,K_dpp_x = x_train_dpps[idx],K_dpp_xs[idx]
        x_dpp = x_train_dpp[get_dpp_subset_from_K(K_dpp_x,batch_size),:]
        
        idx = np.random.permutation(len(K_dpp_xs))[0]
        x_train_dpp,K_dpp_x = x_train_dpps[idx],K_dpp_xs[idx]
        x_dpp2 = x_train_dpp[get_dpp_subset_from_K(K_dpp_x,batch_size),:]

        # Random subset
        r_idx = np.random.permutation(n)[:batch_size]
        x_rand = x_feasible[r_idx,:]

        # Use random subset for recon / DPP subset for latent prior 
        if USE_DPP:
            if np.random.rand() > 0.5: 
                x_recon_in,x_recon_out = x_rand,x_rand
            else:
                x_recon_in,x_recon_out = x_dpp,x_dpp
            x_latent = x_dpp2
        else:
            x_recon_in = x_rand
            x_recon_out = x_rand
            x_latent = x_rand

        # Update 
        recon_loss,wd_loss,d_loss,g_loss = W.update(
            sess,x_recon_in,x_recon_out,x_latent,
               beta=beta,l1_coef=l1_coef,l2_coef=l2_coef,wd_coef=wd_coef,
               lr_recon=lr_recon,lr_d=lr_d*lr_d_weight,lr_g=lr_g)

        # Learning rate schedule
        g_loss_over_d_loss_x = g_loss/d_loss
        if g_loss_over_d_loss_x > 0:
            lr_d_weight = 1.0/g_loss_over_d_loss_x
        else:
            lr_d_weight = 1.0

        if ((it % print_every) == 0) or ((it+1) == max_iter): 
            print ("[%d][%.1f%%] R:[%.3f] WD:[%.3f] D:[%.3f] G:[%.3f]"%
                   (it,it/max_iter*100,recon_loss,wd_loss,d_loss,g_loss))

        if ((it % plot_every) == 0) or ((it+1) == max_iter): 
            print ("[%d][%.1f%%] Plot "%(it,it/max_iter*100))

            ms,ma,tfs = 15,0.5,15
            plt.figure(figsize=(3*4,3))
            # Plot Training Data
            plt.subplot(1,4,1)
            xmin,xmax,ymin,ymax,am = -0.5,+0.5,-0.5,+0.5,0.1
            plt.scatter(
                x_feasible[:,0],x_feasible[:,1],
                c=c,edgecolors=c,s=ms,marker='o',alpha=ma)
            plt.gca().set_aspect('equal', adjustable='box')
            plt.title('Training Data',fontsize=tfs);plt.grid()
            plt.axis([xmin-am,xmax+am,ymin-am,ymax+am])
            # Plot Reconstructed Data
            plt.subplot(1,4,2)
            x_recon = sess.run(W.x_recon,feed_dict={W.ph_x_real:x_feasible})
            plt.scatter(
                x_recon[:,0],x_recon[:,1],
                c=c,edgecolors=c,s=ms,marker='o',alpha=ma)
            plt.gca().set_aspect('equal', adjustable='box')
            plt.title('Reconstructed Data',fontsize=tfs);plt.grid()
            plt.axis([xmin-am,xmax+am,ymin-am,ymax+am])
            # Plot Latent Prior
            plt.subplot(1,4,3)
            xmin,xmax,ymin,ymax,am = -1,+1,-1,+1,0.1
            z_prior = W.S.sampler(1000)
            plt.scatter(z_prior[:,0],z_prior[:,1],c='k',cmap='jet')
            plt.gca().set_aspect('equal', adjustable='box')
            plt.title('Latent Prior',fontsize=tfs);plt.grid()
            plt.axis([xmin-am,xmax+am,ymin-am,ymax+am])
            # Plot Latent of X
            plt.subplot(1,4,4)
            z_real = sess.run(W.z_real,feed_dict={W.ph_x_real:x_feasible})
            plt.scatter(z_real[:,0],z_real[:,1],c=c,cmap='jet')
            plt.gca().set_aspect('equal', adjustable='box')
            plt.title('Latent of X',fontsize=tfs);plt.grid()
            plt.axis([xmin-am,xmax+am,ymin-am,ymax+am])
            plt.show()
        

    
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        