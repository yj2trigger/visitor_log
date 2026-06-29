import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:web_socket_channel/web_socket_channel.dart';

const _channelName = 'arcore_depth_stream';
const _throttleN   = 6;
const _defaultPort = '8765';
const _prefKeyIp   = 'last_ip';

void main() => runApp(const MaterialApp(home: StreamingScreen()));

class StreamingScreen extends StatefulWidget {
  const StreamingScreen({super.key});
  @override
  State<StreamingScreen> createState() => _StreamingScreenState();
}

class _StreamingScreenState extends State<StreamingScreen> {
  final _ipCtrl = TextEditingController();
  WebSocketChannel? _ws;
  bool _isConnected = false;
  int  _frameCount  = 0;
  int  _sentCount   = 0;
  String _status    = '자동 연결 중...';

  @override
  void initState() {
    super.initState();
    const EventChannel(_channelName)
        .receiveBroadcastStream()
        .listen(_onFrame, onError: (e) {
      setState(() => _status = '오류: $e');
    });
    _autoConnect();
  }

  Future<void> _autoConnect() async {
    final prefs = await SharedPreferences.getInstance();
    final savedIp = prefs.getString(_prefKeyIp) ?? '';
    if (savedIp.isEmpty) {
      setState(() => _status = 'IP를 입력하세요');
      return;
    }
    _ipCtrl.text = savedIp;
    _connect();
  }

  void _onFrame(dynamic data) {
    _frameCount++;
    if (!_isConnected || _ws == null) return;
    if (_frameCount % _throttleN != 0) return;
    _ws!.sink.add(data as Uint8List);
    setState(() => _sentCount++);
  }

  Future<void> _connect() async {
    final ip = _ipCtrl.text.trim();
    if (ip.isEmpty) {
      setState(() => _status = 'IP를 입력하세요');
      return;
    }
    try {
      _ws = WebSocketChannel.connect(Uri.parse('ws://$ip:$_defaultPort'));
      final prefs = await SharedPreferences.getInstance();
      await prefs.setString(_prefKeyIp, ip);   // IP 저장
      setState(() {
        _isConnected = true;
        _sentCount   = 0;
        _status      = '$ip:$_defaultPort 연결됨';
      });
    } catch (e) {
      setState(() => _status = '연결 실패: $e');
    }
  }

  void _disconnect() {
    _ws?.sink.close();
    setState(() {
      _ws          = null;
      _isConnected = false;
      _status      = '연결 해제됨';
    });
  }

  @override
  void dispose() {
    _ipCtrl.dispose();
    _ws?.sink.close();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('ARCore Streamer')),
      body: Padding(
        padding: const EdgeInsets.all(20),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            TextField(
              controller: _ipCtrl,
              decoration: const InputDecoration(
                labelText: '노트북 IP',
                hintText: '예: 192.168.0.10',
                border: OutlineInputBorder(),
              ),
              keyboardType: TextInputType.number,
              enabled: !_isConnected,
            ),
            const SizedBox(height: 16),
            ElevatedButton(
              onPressed: _isConnected ? _disconnect : _connect,
              style: ElevatedButton.styleFrom(
                backgroundColor: _isConnected ? Colors.red : Colors.blue,
                padding: const EdgeInsets.symmetric(vertical: 14),
              ),
              child: Text(
                _isConnected ? '연결 해제' : '연결',
                style: const TextStyle(fontSize: 18, color: Colors.white),
              ),
            ),
            const SizedBox(height: 24),
            _InfoTile('상태', _status),
            _InfoTile('수신 프레임', '$_frameCount'),
            _InfoTile('전송 프레임', '$_sentCount  (매 $_throttleN번째)'),
          ],
        ),
      ),
    );
  }
}

class _InfoTile extends StatelessWidget {
  final String label;
  final String value;
  const _InfoTile(this.label, this.value);

  @override
  Widget build(BuildContext context) => Padding(
        padding: const EdgeInsets.symmetric(vertical: 6),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            Text(label, style: const TextStyle(fontWeight: FontWeight.bold)),
            Text(value),
          ],
        ),
      );
}
