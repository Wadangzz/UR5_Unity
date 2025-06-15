using UnityEngine;
using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.IO;
using System.Net;
using Unity.VisualScripting;
using UnityEngine.UIElements;
using UnityEngine.Timeline;

public class UR5 : MonoBehaviour
{
    private TcpListener tcpListener;
    private TcpClient client;
    private StreamReader reader;
    private StreamWriter writer;
    private Thread acceptThread;
    private Thread sendThread;
    private Thread receiveThread;
    private readonly float[] currentAngles = new float[6];
    private readonly float[] jointAngles = new float[6] { 0, 0, 0, 0, 0, 0 };
    private Vector3[] axis = new Vector3[6] { Vector3.down,
                                            Vector3.left,
                                            Vector3.left,
                                            Vector3.left, 
                                            Vector3.down, 
                                            Vector3.left };
    private float[] pos = new float[3] { 0, 0, 0 };
    private float[] quat = new float[4] { 0, 0, 0, 0 };

    private bool isConnected = false;

    [Header("Robots")]
    [SerializeField] private GameObject[] ur5 = new GameObject[7];

    [Serializable]
    public class RobotState
    {
        public float[] joints;
        public float[] position;
        public float[] rotation;
    }

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        tcpListener = new TcpListener(IPAddress.Any, 5000);
        tcpListener.Start();

        acceptThread = new Thread(AcceptClientLoop);
        acceptThread.IsBackground = true;
        acceptThread.Start();
    }
    private float[] JsonToFloatArray(string json)
    {
        // 파싱을 직접 구현 (System.Text.Json 또는 Json.NET 가능)
        json = json.Trim('[', ']');
        string[] tokens = json.Split(',');
        float[] result = new float[tokens.Length];
        for (int i = 0; i < tokens.Length; i++)
            float.TryParse(tokens[i], out result[i]);
        return result;
    }

    void AcceptClientLoop()
    {
        while (true)
        {
            try
            {
                Debug.Log("[SERVER] Waiting for Python client...");
                client = tcpListener.AcceptTcpClient();
                Debug.Log("[SERVER] Python client connected.");

                NetworkStream stream = client.GetStream();
                reader = new StreamReader(stream, Encoding.UTF8);
                writer = new StreamWriter(stream, Encoding.UTF8) { AutoFlush = true };
                isConnected = true;

                sendThread = new Thread(SendLoop);
                receiveThread = new Thread(ReceiveLoop);
                sendThread.Start();
                receiveThread.Start();

                receiveThread.Join(); // 수신 스레드 종료되면 다시 연결 대기
                Debug.LogWarning("[SERVER] Client disconnected. Restarting accept loop...");

                reader?.Close();
                writer?.Close();
                client?.Close();
                isConnected = false;
            }
            catch (Exception ex)
            {
                Debug.LogError($"[SERVER] Connection failed: {ex.Message}");
                Thread.Sleep(1000);
            }
        }
    }

    void SendLoop()
    {
        try
        {
            while (isConnected)
            {
                RobotState state = new RobotState
                {
                    joints = jointAngles,
                    position = pos,
                    rotation = quat
                };
                string msg = JsonUtility.ToJson(state);
                writer.WriteLine(msg);
                Thread.Sleep(50);
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"[SEND] Error: {ex.Message}");
            isConnected = false;
        }
    }

    void ReceiveLoop()
    {
        try
        {
            while (isConnected)
            {
                string json = reader.ReadLine();
                if (!string.IsNullOrEmpty(json))
                {
                    float[] angles = JsonToFloatArray(json);
                    lock (currentAngles)
                    {
                        Array.Copy(angles, currentAngles, 6);
                    }
                }
                else
                {
                    Debug.LogWarning("[RECEIVE] Python disconnected (ReadLine returned null)");
                    isConnected = false;
                }
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"[RECEIVE] Error: {ex.Message}");
            isConnected = false;
        }
    }


    // Update is called once per frame
    void Update()
    {
        lock (currentAngles)
        {
            Array.Copy(currentAngles, jointAngles, 6);
        }
        ////Debug.Log($"[UR5 angles] {string.Join(", ", jointAngles)}");
        //ur5[1].transform.localRotation = Quaternion.Euler(0, -jointAngles[0], 0);
        //ur5[2].transform.localRotation = Quaternion.Euler(-jointAngles[1], 0, 0);
        //ur5[3].transform.localRotation = Quaternion.Euler(-jointAngles[2], 0, 0);
        //ur5[4].transform.localRotation = Quaternion.Euler(-jointAngles[3], 0, 0);
        //ur5[5].transform.localRotation = Quaternion.Euler(0, -jointAngles[4], 0);
        //ur5[6].transform.localRotation = Quaternion.Euler(-jointAngles[5], 0, 0);
        for (int i = 0; i < 6;i++)
        {
            ur5[i+1].transform.localRotation = Quaternion.AngleAxis(jointAngles[i],axis[i]);
        }
        pos = new float[3] { ur5[6].transform.position[0],
                            ur5[6].transform.position[2], 
                            ur5[6].transform.position[1] };
        quat = new float[4] { -ur5[6].transform.rotation.x,
                            -ur5[6].transform.rotation.z,
                            -ur5[6].transform.rotation.y,
                            ur5[6].transform.rotation.w };

        Debug.Log($"End-Effector : {string.Join(", ", pos)}, {string.Join(", ", quat)}");
        Debug.Log($"JointAngles : {String.Join(", ", jointAngles)}");
    }
    void OnApplicationQuit()
    {
        reader?.Close();
        tcpListener.Stop();
        try { acceptThread?.Interrupt(); } catch { }
        try { sendThread?.Interrupt(); } catch { }
        try { receiveThread?.Interrupt(); } catch { }
    }
}
