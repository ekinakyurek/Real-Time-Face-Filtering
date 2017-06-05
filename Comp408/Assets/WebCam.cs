using UnityEngine;
using System.Collections;
using UnityEngine.UI;
using UnityEngine.Windows;
using System.IO;

public class WebCam : MonoBehaviour {

	WebCamTexture _webcamtex;

	void Start() {

	_webcamtex = new WebCamTexture();
	Renderer _renderer = GetComponent<Renderer>();
	_webcamtex.requestedFPS = 24;
	_renderer.material.mainTexture = _webcamtex;
	_webcamtex.Play();

    }

	
	// Update is called once per frame
	void Update () {


//	if(Input.GetKeyDown(KeyCode.A))
//		{
//		StartCoroutine(CaptureTextureAsPNG());
//		}
	}

	#region CaptureSnap
//	IEnumerator CaptureTextureAsPNG()
//	{
//	int size;
//	yield return new WaitForEndOfFrame();
//	Texture2D _TextureFromCamera = new Texture2D(GetComponent<Renderer>().material.mainTexture.width,
//	GetComponent<Renderer>().material.mainTexture.height);
//	_TextureFromCamera.SetPixels((GetComponent<Renderer>().material.mainTexture as WebCamTexture).GetPixels());
//	_TextureFromCamera.Apply();
//	byte [] bytes = _TextureFromCamera.EncodeToPNG();
//
//	size = bytes.Length;
//
//
//	string filePath = "SavedScreen1.png";
//	File.WriteAllBytes(filePath, bytes);
//
//}
#endregion 
}