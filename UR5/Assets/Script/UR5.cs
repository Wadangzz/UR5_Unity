using UnityEngine;
using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.IO;
using System.Net;
using Unity.VisualScripting;

public class UR5 : MonoBehaviour
{
    private TcpListener tcpListener;
    private TcpClient client;
    private StreamReader reader;
    private StreamWriter writer;
    private Thread sendThread;
    private Thread receiveThread;
    private readonly float[] currentAngles = new float[6];
    private float[] jointAngles = new float[6] { 0, 0, 0, 0, 0, 0 };
    private float[] pos = new float[3] { 0, 0, 0 };
    private float[] endEffector = new float[3] { 0, 0, 0 };

    private bool isConnected = false;

    [Header("Robots")]
    [SerializeField] private GameObject[] ur5 = new GameObject[7];

    [Header("Endeffector")]
    [SerializeField] private GameObject end;

    [Serializable]
    public class RobotState
    {
        public float[] joints;
        public float[] position;
        public float[] rotation;
    }

    public static class RotationUtils
    {
        public static Vector3 WorldEulerZYX(Transform t)
        {
            // 월드 기준 회전 쿼터니언
            Quaternion worldRot = t.rotation;

            // 쿼터니언을 오일러 각도로 변환 (Unity는 ZYX 기준)
            Vector3 euler = worldRot.eulerAngles;

            // 오일러 각도 정규화 (-180 ~ 180 범위)
            for (int i = 0; i < 3; i++)
            {
                if (euler[i] > 180f)
                    euler[i] -= 360f;
            }

            // 결과: (Roll, Pitch, Yaw)
            return euler;
        }
    }

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        tcpListener = new TcpListener(IPAddress.Any, 5000);
        tcpListener.Start();

        Thread acceptThread = new Thread(AcceptClientLoop);
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

                // 정리
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
                    rotation = endEffector
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
        //Debug.Log($"[UR5 angles] {string.Join(", ", jointAngles)}");
        ur5[1].transform.localRotation = Quaternion.Euler(0, -jointAngles[0], 0);
        ur5[2].transform.localRotation = Quaternion.Euler(-jointAngles[1], 0, 0);
        ur5[3].transform.localRotation = Quaternion.Euler(-jointAngles[2], 0, 0);
        ur5[4].transform.localRotation = Quaternion.Euler(-jointAngles[3], 0, 0);
        ur5[5].transform.localRotation = Quaternion.Euler(0, -jointAngles[4], 0);
        ur5[6].transform.localRotation = Quaternion.Euler(-jointAngles[5], 0, 0);

        Vector3 endEuler = RotationUtils.WorldEulerZYX(end.transform);

        pos = new float[3] { ur5[6].transform.position[0], ur5[6].transform.position[2], ur5[6].transform.position[1] };
        endEffector = new float[3] { endEuler.x, endEuler.y, endEuler.z };
        //endEff = new float[3] { end.transform.eulerAngles[0], end.transform.eulerAngles[2], end.transform.eulerAngles[1] };


        //for (int i = 0; i < 3; i++)
        //{
        //    if (endEffector[i] > 1e-3)
        //    {
        //        endEffector[i] = 360.0f - endEffector[i];
        //    }
        //    else
        //    {
        //        continue;
        //    }
        //}

        Debug.Log($"End-Effector : {string.Join(", ", pos)}, {String.Join(", ", endEffector)}");
        Debug.Log($"Jointangles : {String.Join(", ", jointAngles)}");
    }
    void OnApplicationQuit()
    {
        reader?.Close();
        tcpListener.Stop();
        try { sendThread?.Interrupt(); } catch { }
        try { receiveThread?.Interrupt(); } catch { }
    }
}


//private void Send_ReceiveData()
//{
//    try
//    {
//        tcpListener = new TcpListener(IPAddress.Any,5000);
//        tcpListener.Start();
//        Debug.Log("Waiting for Python client...");

//        while (!isConnected)
//        {
//            isConnecting = true;
//            using (client = tcpListener.AcceptTcpClient())
//            using (reader = new StreamReader(client.GetStream(), Encoding.UTF8))
//            using (writer = new StreamWriter(client.GetStream(), Encoding.UTF8) { AutoFlush = true })
//            {
//                Debug.Log("Python connected.");
//                isConnected = true;
//                isConnecting = false;
//                while (isConnected)
//                {
//                    if (!sentOnce)
//                    {
//                        writer.WriteLine("[0,0,0,0,0,0]");
//                        sentOnce = true;
//                    }
//                    else
//                    {
//                        // JSON으로 현재 각도 전송
//                        string response = "[" + string.Join(",", jointAngles) + "]";
//                        writer.WriteLine(response);
//                    }

//                   Thread.Sleep(50);
//                    string json = reader.ReadLine();
//                    if (!string.IsNullOrEmpty(json))
//                    {
//                        float[] angles = JsonToFloatArray(json);
//                        if (angles.Length == currentAngles.Length)
//                        {
//                            lock (currentAngles)
//                            {
//                                Array.Copy(angles, currentAngles, angles.Length);
//                            }
//                        }
//                        //Debug.Log("Received angles: " + string.Join(", ", angles));
//                    }
//                    else
//                    {
//                        Debug.LogWarning(" Python disconnected (ReadLine returned null)");
//                        isConnected = false;
//                        break;  // 내부 루프 탈출 → 재연결 시도 가능
//                    } 
//                }
//            }
//        }
//    }
//    catch (Exception e)
//    {
//        Debug.LogError("Connection failed: " + e.Message);
//        try
//        {
//            Thread.Sleep(1000);
//        }
//        catch (ThreadInterruptedException)
//        {
//            Debug.Log("Receive thread interrupted during sleep. Exiting thread...");
//            return; // 스레드 안전 종료
//        }
//    }    
//}

