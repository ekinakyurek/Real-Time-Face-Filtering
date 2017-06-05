using UnityEngine; 
using System.Collections;
using System.Threading; 
using System.Net.Sockets; 
using System.IO;
using System.Linq;
using System.Globalization;
using System;

public class Glasses : MonoBehaviour {
	private bool debug_mode = false;
	private bool mRunning;
	string msg = "";
	Thread mThread;
 	TcpListener tcp_Listener = null;
	private string temp;
	public string input_data;

	private int[] face_borders = new int[4];
	private Vector2[] image_points = new Vector2[6];
	private Vector2[] temp_image_points = new Vector2[6];
	private Vector2 nose_bridge = new Vector2();
	private Vector2 nose_bridge_temp = new Vector2();
	private Vector3 euler_angels = new Vector3();
	private float input_size; 

	private Quaternion input_rotation;
	private float rot_x;
	private float rot_y;
	private float rot_z;

	private Vector3 input_position;
	private float pos_x;
	private float pos_y;


	void Start()
 {
     mRunning = true;
     ThreadStart ts = new ThreadStart(SayHello);
     mThread = new Thread(ts);
	 mThread.Start();
		if (debug_mode)print("Thread done...");



 }
		

 #region Socket
	 public void stopListening()
	{
     mRunning = false;
	}


	void SayHello()
	{
     try
     {
         tcp_Listener = new TcpListener(1234);
         tcp_Listener.Start();
		 if (debug_mode)print("Server Start");
         while (mRunning)
         {
             // check if new connections are pending, if not, be nice and sleep 100ms
				if (!tcp_Listener.Pending()){
                 Thread.Sleep(1000);
             }
             else
             {
                /// print("1");
                 TcpClient client = tcp_Listener.AcceptTcpClient();
                 //print("2");
                 NetworkStream ns = client.GetStream();
				//print("3");
				 StreamReader reader = new StreamReader(ns);
                // print("4");
					while(msg !="stop" ){
						msg = reader.ReadLine();
					    string[] data_array = msg.Split(']');

						face_borders = data_array[0].Substring(1).Split(',').Select(x=>int.Parse(x)).ToArray();

						string[] image_points_str = data_array[1].Substring(2).Split(')');
						string[] xy = image_points_str[0].Substring(1).Split(',');

						temp_image_points[0] = new Vector2(float.Parse(xy[0]),float.Parse(xy[1]));
						
						for(int i = 1; i< image_points.Length; i++ ){
							string[] x = image_points_str[i].Substring(2).Split(',');
							temp_image_points[i] = new Vector2(float.Parse(x[0]),float.Parse(x[1]));
						}

						string[] nose_bridge_str = data_array[2].Substring(2).Split(',');
						nose_bridge_temp.x = float.Parse(nose_bridge_str[0]);
						nose_bridge_temp.y = float.Parse(nose_bridge_str[1]);


						string[] euler_angels_str = data_array[3].Substring(2).Split(',');

						euler_angels.x = float.Parse(euler_angels_str[0]);
						if(euler_angels.x >= 0.0f){
							euler_angels.x = (euler_angels.x - 180.0f) * -1 ;
						}else{
							euler_angels.x = (euler_angels.x + 180.0f) * -1;
						}
						euler_angels.y = float.Parse(euler_angels_str[1]);
						euler_angels.z = float.Parse(euler_angels_str[2]);


						//print(euler_angels);
						//print("message comes");
                 }

				reader.Close();
		 		client.Close();

             }
         }
		}
     catch (ThreadAbortException)
     {
         print("exception");
     }
		finally
     {
         mRunning = false;
         tcp_Listener.Stop();
     }
	}



 void OnApplicationQuit()
 {
		// stop listening thread stopListening();
     // wait fpr listening thread to terminate (max. 500ms)
     mThread.Join(500);
 }
 #endregion


		


	void FixedUpdate(){
		UpdateGlasses ();

 	}


	void UpdateGlasses(){
		if (temp_image_points [0].magnitude != 0.0f && nose_bridge != nose_bridge_temp) {
			print ("Updated");
			image_points = temp_image_points;
			nose_bridge = nose_bridge_temp;
			//input_size = System.Convert.ToSingle();
			Vector2 distance = (image_points [2] - image_points [3]);
			float scale = distance.magnitude * 1.3f;
			//scale = scale / Convert.ToSingle(Math.Cos (euler_angels.y));
			if (debug_mode)
				print ("scale:" + scale);
			this.transform.localScale = new Vector3 (scale, scale, scale);
			//
			pos_x = nose_bridge.x;
			pos_y = -1 * nose_bridge.y;

			if (debug_mode)
				print ("posx:" + pos_x + " posy:" + pos_y);
			// input_position = new Vector3(pos_x,pos_y,this.transform.position.z);
			this.transform.position = new Vector3 (pos_x, pos_y, this.transform.position.z);
			//		print(input_position);
			//
			//			rot_x = Convert.ToSingle((Math.Atan(distance.y/distance.x)/Math.PI)*180.0);
			//		  	Vector2 nose_chin_distance = image_points [0] - image_points [1];
			//			rot_y = 0.0f;
			//rot_z = Convert.ToSingle((Math.Atan(nose_chin_distance.x/nose_chin_distance.y/Math.PI)*180.0));

			if (debug_mode)
				print ("rotx:" + euler_angels.x + " roty:" + euler_angels.y + " rotz:" + -1 * euler_angels.z);
			//
			//input_rotation = Quaternion.Euler(new Vector3(euler_angels.x, euler_angels.y, -1*euler_angels.z));
			this.transform.rotation = Quaternion.Euler (new Vector3 (euler_angels.x, euler_angels.y, -1 * euler_angels.z));
		}
	}
 }