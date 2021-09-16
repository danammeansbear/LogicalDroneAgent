using System.Collections.Concurrent;
using System.Threading;
using NetMQ;
using UnityEngine;
using NetMQ.Sockets;

public class NetMqListener
{
    private readonly Thread _listenerWorker;

    private bool _listenerCancelled;

    public delegate void MessageDelegate(string message);

    public delegate void TargetDelegate(string targetLocation);

    public delegate void LockedTargetDelegate(string lockedTargetLocation);

    private readonly MessageDelegate _messageDelegate;

    //private readonly TargetDelegate _targetLocationDelegate;

    //private readonly LockedTargetDelegate _lockedTargetDelegate;

    private readonly ConcurrentQueue<string> _messageQueue = new ConcurrentQueue<string>();
    //private readonly ConcurrentQueue<string> _targetLocationQueue = new ConcurrentQueue<string>();
    //private readonly ConcurrentQueue<string> _lockedTargetQueue = new ConcurrentQueue<string>();

    private void ListenerWork()
    {
        AsyncIO.ForceDotNet.Force();
        using (var subSocket = new SubscriberSocket())
        {
            subSocket.Options.ReceiveHighWatermark = 1000;
            subSocket.Connect("tcp://localhost:12345");
            subSocket.Subscribe("");
            while (!_listenerCancelled)
            {
                string frameString;
                if (!subSocket.TryReceiveFrameString(out frameString)) continue;
                Debug.Log(frameString);
                _messageQueue.Enqueue(frameString);
            }
            subSocket.Close();
        }
        NetMQConfig.Cleanup();
    }

    public void Update()
    {
        while (!_messageQueue.IsEmpty)
        {
            string message;
            string targetLocation;
            string lockedTargetLocation;
            if (_messageQueue.TryDequeue(out message))
            {
                _messageDelegate(message);
            }/*
            if (_targetLocationQueue.TryDequeue(out targetLocation))
            {
                _targetLocationDelegate(targetLocation);
            }
            if (_lockedTargetQueue.TryDequeue(out lockedTargetLocation))
            {
                _lockedTargetDelegate(lockedTargetLocation);
            }*/
            else
            {
                break;
            }
        }
    }

    public NetMqListener(MessageDelegate messageDelegate)
    {
        _messageDelegate = messageDelegate;
        _listenerWorker = new Thread(ListenerWork);
    }
    /*
    public NetMqListener(TargetDelegate targetDelegate)
    {
        _targetLocationDelegate = targetDelegate;
        _listenerWorker = new Thread(ListenerWork);
    }
    public NetMqListener(LockedTargetDelegate lockedTargetDelegate)
    {
        _lockedTargetDelegate = lockedTargetDelegate;
        _listenerWorker = new Thread(ListenerWork);
    }
    */
    public void Start()
    {
        _listenerCancelled = false;
        _listenerWorker.Start();
    }

    public void Stop()
    {
        _listenerCancelled = true;
        _listenerWorker.Join();
    }
}

public class ClientObject : MonoBehaviour
{
    private NetMqListener _netMqListener;

    private void HandleMessage(string message)
    //private void HandleMessage(string message, string targetLocation, string lockedTargetLocation)
    {
        var droneLocationsplittedStrings = message.Split(' ');
        if (droneLocationsplittedStrings.Length != 3) return;
        var xd = float.Parse(droneLocationsplittedStrings[0]);
        var yd = float.Parse(droneLocationsplittedStrings[1]);
        var zd = float.Parse(droneLocationsplittedStrings[2]);
        transform.position = new Vector3(xd, yd, zd);
       /*
        var targetLocationsplittedStrings = targetLocation.Split(' ');
        if (targetLocationsplittedStrings.Length != 3) return;
        var xl = float.Parse(targetLocationsplittedStrings[0]);
        var yl = float.Parse(targetLocationsplittedStrings[1]);
        var zl = float.Parse(targetLocationsplittedStrings[2]);
        transform.position = new Vector3(xl, yl, zl);

        var lockedTargetLocationsplittedStrings = lockedTargetLocation.Split(' ');
        if (lockedTargetLocationsplittedStrings.Length != 3) return;
        var xll = float.Parse(lockedTargetLocationsplittedStrings[0]);
        var yll = float.Parse(lockedTargetLocationsplittedStrings[1]);
        var zll = float.Parse(lockedTargetLocationsplittedStrings[2]);
        transform.position = new Vector3(xll, yll, zll);
       */



    }

    private void Start()
    {
        _netMqListener = new NetMqListener(HandleMessage);
        _netMqListener.Start();
    }

    private void Update()
    {
        _netMqListener.Update();
    }

    private void OnDestroy()
    {
        _netMqListener.Stop();
    }
}
