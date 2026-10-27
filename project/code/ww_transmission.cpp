#/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — 摄像头模块
 * 版权所有 (c) 2025 Blockingsys
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * 本文件是 Wuwu 开源库 的一部分。
 *
 * 本文件按照 GNU 通用公共许可证 第3版（GPLv3）或您选择的任何后续版本的条款授权。
 * 您可以在遵守 GPL-3.0 许可条款的前提下，自由地使用、复制、修改和分发本文件及其衍生作品。
 * 在分发本文件或其衍生作品时，必须以相同的许可证（GPL-3.0）对源代码进行授权并随附许可证副本。
 *
 * 本软件按“原样”提供，不对适销性、特定用途适用性或不侵权做任何明示或暗示的保证。
 * 有关更多细节，请参阅 GNU 官方许可证文本： https://www.gnu.org/licenses/gpl-3.0.html
 *
 * 注：本注释为 GPL-3.0 许可证的中文说明与摘要，不构成法律意见。正式许可以 GPL 原文为准。
 * LICENSE 副本通常位于项目根目录的 LICENSE 文件或 libraries 文件夹下；若未找到，请访问上方链接获取。
 *
 * 额外说明：
 * - 本项目可能包含第三方组件，各组件的版权与许可以其各自随附的 LICENSE 为准；
 * - 分发、修改本文件时请保留本版权与许可声明以尊重原作者权利；
 * - 本文档为中文译述/摘要，英文许可证文本为法律权威版。
 *
 * 文件名称：ww_transmission.cc
 * 所属模块：wuwu_library
 * 功能描述：摄像头服务器，用于 MJPEG 流与 HTTP 接口
 * 版本信息：详见 libraries/doc/version
 * 开发环境：Linux，GCC / Clang，OpenCV，V4L2
 * 联系/主页：请参阅项目 README
 *
 * 修改记录：
 * 日期        作者              说明
 * 2025-12-16  Blockingsys    添加 GPL-3.0 中文许可头
 ********************************************************************************************************************/

#include "ww_transmission.h"
#include "dataset_capture.hpp"

namespace
{
std::string QueryParameter(const std::string &path, const std::string &key)
{
    const std::size_t query_position = path.find('?');
    if (query_position == std::string::npos)
        return "";
    std::size_t position = query_position + 1;
    while (position < path.size())
    {
        const std::size_t end = path.find('&', position);
        const std::string item = path.substr(
            position, end == std::string::npos ? std::string::npos : end - position);
        const std::size_t equals = item.find('=');
        if (equals != std::string::npos && item.substr(0, equals) == key)
            return item.substr(equals + 1);
        if (end == std::string::npos)
            break;
        position = end + 1;
    }
    return "";
}

std::string RequestPathOnly(const std::string &path)
{
    const std::size_t query_position = path.find('?');
    return path.substr(0, query_position);
}
}

// HTML查看器内容
const char* viewer_html = R"HTML(
<!DOCTYPE html> 
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>摄像头实时查看</title>
    <style>
        body { margin: 0; padding: 20px; background: #1a1a1a; font-family: Arial, sans-serif; }
        .container { max-width: 1200px; margin: 0 auto; background: #2d2d2d; border-radius: 10px; padding: 20px; box-shadow: 0 5px 20px rgba(0,0,0,0.5); }
        h1 { color: #fff; text-align: center; margin-bottom: 20px; }
        #stream { width: 100%; border-radius: 8px; background: #000; }
        .controls { margin-top: 20px; text-align: center; }
        button { background: #4CAF50; color: white; border: none; padding: 12px 24px; margin: 5px; border-radius: 5px; cursor: pointer; font-size: 16px; }
        button:hover { background: #45a049; }
        .snapshot-btn { background: #2196F3; }
        .snapshot-btn:hover { background: #0b7dda; }
        .info { color: #aaa; margin-top: 15px; font-size: 14px; line-height: 1.6; }
        .hint { color: #f5a623; font-size: 13px; margin-top: 8px; }
        .status { display: inline-block; width: 10px; height: 10px; border-radius: 50%; background: #4CAF50; margin-right: 8px; animation: pulse 2s infinite; }
        @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.5; } }
        .filename-config { margin-top: 15px; text-align: center; }
        .filename-config label { color: #aaa; font-size: 14px; margin-right: 10px; }
        .filename-config input { background: #1a1a1a; color: #fff; border: 1px solid #555; padding: 8px 12px; border-radius: 5px; font-size: 14px; width: 200px; }
        .filename-config input:focus { outline: none; border-color: #4CAF50; }
        .dataset-panel { margin-top: 22px; padding-top: 20px; border-top: 1px solid #505050; color: #eee; }
        .dataset-panel h2 { margin: 0 0 14px; font-size: 20px; letter-spacing: 0; }
        .label-group { display: grid; grid-template-columns: 90px minmax(0, 1fr); gap: 10px; align-items: start; margin-bottom: 10px; }
        .label-group-name { color: #aaa; padding-top: 9px; font-size: 14px; }
        .label-options { display: flex; flex-wrap: wrap; gap: 8px; }
        .label-button { background: #424242; padding: 9px 14px; margin: 0; font-size: 14px; border: 1px solid #666; }
        .label-button:hover { background: #505050; }
        .label-button.selected { background: #1976D2; border-color: #64B5F6; }
        .label-button:disabled, .capture-actions button:disabled { opacity: 0.45; cursor: not-allowed; }
        .capture-actions { display: flex; flex-wrap: wrap; gap: 8px; margin-top: 16px; }
        .capture-actions button { margin: 0; padding: 10px 16px; }
        .capture-actions .burst { background: #00897B; }
        .capture-actions .cancel { background: #C62828; }
        .capture-state { margin-top: 16px; display: grid; grid-template-columns: repeat(4, minmax(100px, 1fr)); gap: 8px; }
        .capture-metric { background: #242424; border: 1px solid #444; border-radius: 5px; padding: 10px; min-width: 0; }
        .capture-metric span { display: block; color: #999; font-size: 12px; margin-bottom: 4px; }
        .capture-metric strong { display: block; overflow-wrap: anywhere; font-size: 15px; }
        #captureProgress { width: 100%; height: 12px; margin-top: 12px; accent-color: #1976D2; }
        #captureMessage { min-height: 20px; margin-top: 10px; color: #bdbdbd; overflow-wrap: anywhere; }
        #captureMessage.error { color: #ff8a80; }
        #capturePath { color: #90CAF9; font-family: monospace; font-size: 12px; }
        @media (max-width: 700px) {
            body { padding: 8px; }
            .container { padding: 12px; }
            .label-group { grid-template-columns: 1fr; gap: 5px; }
            .label-group-name { padding-top: 0; }
            .capture-state { grid-template-columns: repeat(2, minmax(0, 1fr)); }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🎥 摄像头实时查看 <span class="status"></span></h1>
        <img id="stream" src="/stream" alt="摄像头画面">
        <div class="controls">
            <button class="snapshot-btn" onclick="takeSnapshot()">📸 拍照保存</button>
            <button onclick="reconnect()">🔄 重新连接</button>
            <button onclick="toggleFullscreen()">⛶ 全屏</button>
        </div>
        <div class="filename-config">
            <label>文件名前缀:</label>
            <input type="text" id="filenamePrefix" value="snapshot" placeholder="snapshot" />
            <span style="color: #777; font-size: 12px; margin-left: 10px;">格式: 前缀_年月日_时分秒.jpg</span>
        </div>
        <section class="dataset-panel" aria-labelledby="datasetCaptureTitle">
            <h2 id="datasetCaptureTitle">八类标注采集</h2>
            <div id="labelGroups"></div>
            <div class="capture-actions">
                <button id="singleCaptureButton" type="button" onclick="startDatasetCapture('single')" disabled>单拍（先选类别）</button>
                <button id="burstCaptureButton" class="burst" type="button" onclick="startDatasetCapture('burst')" disabled>5 秒连拍（先选类别）</button>
                <button id="cancelCaptureButton" class="cancel" type="button" onclick="cancelDatasetCapture()" disabled>取消</button>
            </div>
            <div class="capture-state">
                <div class="capture-metric"><span>类别</span><strong id="captureLabel">--</strong></div>
                <div class="capture-metric"><span>状态</span><strong id="capturePhase">idle</strong></div>
                <div class="capture-metric"><span>已保存 / 跳过</span><strong id="captureCounts">0 / 0</strong></div>
                <div class="capture-metric"><span>剩余</span><strong id="captureRemaining">--</strong></div>
            </div>
            <progress id="captureProgress" value="0" max="1"></progress>
            <div id="captureMessage"></div>
            <div id="capturePath"></div>
        </section>
        <div class="info">
            <p>• 点击"拍照保存"下载摄像头原始JPEG图片 • 支持全屏查看 • 视频流: <span id="url"></span></p>
            <p>• 快捷键: 按 <strong>K</strong> 键快速拍照 • 按 <strong>F</strong> 键全屏</p>
            <p>• 延迟: <strong><span id="latency">--</span></strong> • 实时帧率: <strong><span id="fps">--</span></strong></p>
            <p id="clock-hint" class="hint"></p>
        </div>
    </div>
    <script>
        document.getElementById('url').textContent = window.location.origin + '/stream';
        const img = document.getElementById('stream');
        let datasetLabels = [];
        let selectedDatasetLabel = '';
        let captureRequestPending = false;
        
        // 加载保存的文件名前缀
        const savedPrefix = localStorage.getItem('filenamePrefix') || 'snapshot';
        document.getElementById('filenamePrefix').value = savedPrefix;
        
        // 监听文件名前缀变化，自动保存
        document.getElementById('filenamePrefix').addEventListener('change', function() {
            const prefix = this.value.trim() || 'snapshot';
            localStorage.setItem('filenamePrefix', prefix);
            console.log('文件名前缀已保存:', prefix);
        });
        
        function takeSnapshot() {
            // 获取用户自定义的文件名前缀
            const prefix = document.getElementById('filenamePrefix').value.trim() || 'snapshot';
            
            // 生成带时间戳的文件名
            const now = new Date();
            const year = now.getFullYear();
            const month = String(now.getMonth() + 1).padStart(2, '0');
            const day = String(now.getDate()).padStart(2, '0');
            const hour = String(now.getHours()).padStart(2, '0');
            const minute = String(now.getMinutes()).padStart(2, '0');
            const second = String(now.getSeconds()).padStart(2, '0');
            const filename = `${prefix}_${year}${month}${day}_${hour}${minute}${second}.jpg`;
            
            // 创建隐藏的下载链接，将前缀通过URL参数传递给后端
            const a = document.createElement('a');
            a.href = `/snapshot?prefix=${encodeURIComponent(prefix)}`;
            a.download = filename;
            a.style.display = 'none';
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
            
            // 显示提示（可选）
            const notification = document.createElement('div');
            notification.textContent = '✓ 正在下载: ' + filename;
            notification.style.cssText = 'position:fixed;top:20px;right:20px;background:#4CAF50;color:white;padding:15px 25px;border-radius:5px;box-shadow:0 2px 10px rgba(0,0,0,0.3);z-index:9999;';
            document.body.appendChild(notification);
            setTimeout(() => document.body.removeChild(notification), 3000);
        }
        
        function reconnect() {
            img.src = '/stream?t=' + new Date().getTime();
        }
        
        function toggleFullscreen() {
            if (!document.fullscreenElement) {
                img.requestFullscreen();
            } else {
                document.exitFullscreen();
            }
        }

        function businessDisplayName(business) {
            if (business === 'supplies') return '物资';
            if (business === 'vehicle') return '载具';
            if (business === 'weapon') return '武器';
            return business;
        }

        function updateCaptureActionLabels() {
            const selected = datasetLabels.find((label) => label.id === selectedDatasetLabel);
            document.getElementById('singleCaptureButton').textContent =
                selected ? `单拍 · ${selected.display}` : '单拍（先选类别）';
            document.getElementById('burstCaptureButton').textContent =
                selected ? `5 秒连拍 · ${selected.display}` : '5 秒连拍（先选类别）';
        }

        function renderDatasetLabels() {
            const root = document.getElementById('labelGroups');
            root.textContent = '';
            const businesses = ['supplies', 'vehicle', 'weapon'];
            businesses.forEach((business) => {
                const labels = datasetLabels.filter((label) => label.business === business);
                const group = document.createElement('div');
                group.className = 'label-group';
                const name = document.createElement('div');
                name.className = 'label-group-name';
                name.textContent = businessDisplayName(business);
                const options = document.createElement('div');
                options.className = 'label-options';
                labels.forEach((label) => {
                    const button = document.createElement('button');
                    button.type = 'button';
                    button.className = 'label-button';
                    button.dataset.label = label.id;
                    button.textContent = label.display;
                    button.onclick = () => selectDatasetLabel(label.id);
                    options.appendChild(button);
                });
                group.appendChild(name);
                group.appendChild(options);
                root.appendChild(group);
            });
            if (datasetLabels.some((label) => label.id === selectedDatasetLabel)) {
                selectDatasetLabel(selectedDatasetLabel);
            } else {
                selectedDatasetLabel = '';
                document.querySelectorAll('.label-button').forEach((button) => {
                    button.classList.remove('selected');
                });
                document.getElementById('captureLabel').textContent = '--';
                updateCaptureActionLabels();
            }
        }

        function selectDatasetLabel(labelId) {
            if (!datasetLabels.some((label) => label.id === labelId)) return;
            selectedDatasetLabel = labelId;
            document.querySelectorAll('.label-button').forEach((button) => {
                button.classList.toggle('selected', button.dataset.label === labelId);
            });
            const selected = datasetLabels.find((label) => label.id === labelId);
            document.getElementById('captureLabel').textContent = selected ? selected.display : '--';
            updateCaptureActionLabels();
        }

        async function loadDatasetLabels() {
            const response = await fetch('/dataset-capture/labels', {cache: 'no-store'});
            if (!response.ok) throw new Error('标签接口返回 ' + response.status);
            const data = await response.json();
            datasetLabels = Array.isArray(data.labels) ? data.labels : [];
            if (datasetLabels.length !== 8) throw new Error('标签数量不是 8');
            renderDatasetLabels();
        }

        async function datasetCommand(url) {
            captureRequestPending = true;
            try {
                const response = await fetch(url, {method: 'POST', cache: 'no-store'});
                const data = await response.json();
                if (!response.ok) throw new Error(data.error || ('请求失败 ' + response.status));
                renderDatasetStatus(data);
            } finally {
                captureRequestPending = false;
                updateDatasetCaptureStatus();
            }
        }

        async function startDatasetCapture(mode) {
            if (!selectedDatasetLabel || captureRequestPending) return;
            try {
                await datasetCommand(`/dataset-capture/start?label=${encodeURIComponent(selectedDatasetLabel)}&mode=${mode}`);
            } catch (error) {
                showCaptureError(error.message);
            }
        }

        async function cancelDatasetCapture() {
            if (captureRequestPending) return;
            try {
                await datasetCommand('/dataset-capture/cancel');
            } catch (error) {
                showCaptureError(error.message);
            }
        }

        function showCaptureError(message) {
            const element = document.getElementById('captureMessage');
            element.textContent = message;
            element.classList.add('error');
        }

        function renderDatasetStatus(status) {
            const active = Boolean(status.active);
            const selected = datasetLabels.find((label) => label.id === selectedDatasetLabel);
            document.getElementById('captureLabel').textContent =
                active && status.displayLabel ? status.displayLabel : (selected ? selected.display : '--');
            document.getElementById('capturePhase').textContent = status.phase || 'idle';
            document.getElementById('captureCounts').textContent = `${Number(status.saved) || 0} / ${Number(status.skipped) || 0}`;
            document.getElementById('captureRemaining').textContent = active ? `${(Math.max(0, Number(status.remainingMs) || 0) / 1000).toFixed(1)} s` : '--';
            const progress = document.getElementById('captureProgress');
            progress.max = Math.max(1, Number(status.target) || 1);
            progress.value = Math.min(progress.max, Number(status.scheduled) || 0);
            const message = document.getElementById('captureMessage');
            message.textContent = status.lastMessage || '';
            message.classList.toggle('error', status.phase === 'error');
            document.getElementById('capturePath').textContent = status.sessionPath || '';
            document.getElementById('singleCaptureButton').disabled =
                active || captureRequestPending || !selectedDatasetLabel;
            document.getElementById('burstCaptureButton').disabled =
                active || captureRequestPending || !selectedDatasetLabel;
            document.getElementById('cancelCaptureButton').disabled = !active || captureRequestPending;
            document.querySelectorAll('.label-button').forEach((button) => {
                button.disabled = false;
            });
        }

        async function updateDatasetCaptureStatus() {
            try {
                const response = await fetch('/dataset-capture/status', {cache: 'no-store'});
                if (!response.ok) throw new Error('状态接口返回 ' + response.status);
                renderDatasetStatus(await response.json());
            } catch (error) {
                showCaptureError(error.message);
            }
        }

        // 键盘快捷键
        document.addEventListener('keydown', function(event) {
            // 如果焦点在输入框上，不触发快捷键
            const activeElement = document.activeElement;
            if (activeElement && (activeElement.tagName === 'INPUT' || activeElement.tagName === 'TEXTAREA')) {
                return;
            }
            
            // K键 - 拍照保存
            if (event.key === 'k' || event.key === 'K') {
                event.preventDefault();
                takeSnapshot();
            }
            // F键 - 全屏切换
            else if (event.key === 'f' || event.key === 'F') {
                event.preventDefault();
                toggleFullscreen();
            }
            // R键 - 重新连接
            else if (event.key === 'r' || event.key === 'R') {
                event.preventDefault();
                reconnect();
            }
        });

        async function updateStats() {
            try {
                const response = await fetch('/stats');
                if (!response.ok) throw new Error('stats fetch failed');
                const data = await response.json();
                const latencyEl = document.getElementById('latency');
                const fpsEl = document.getElementById('fps');
                const hintEl = document.getElementById('clock-hint');
                hintEl.textContent = '';

                const captureTs = Number(data.latestCaptureTsMs) || 0;
                const serverTs = Number(data.serverTsMs) || 0;
                const browserNow = Date.now();

                if (captureTs && serverTs) {
                    const internalLatency = Math.max(0, serverTs - captureTs);
                    const clockOffset = browserNow - serverTs;
                    const networkLatency = Math.max(0, clockOffset);
                    if (Math.abs(clockOffset) > 2000) {
                        latencyEl.textContent = internalLatency + ' ms (板载)';
                        hintEl.textContent = '⚠️ 开发板时钟未和电脑同步，浏览器显示的总延迟会偏大。';
                    } else {
                        const endToEnd = internalLatency + networkLatency;
                        latencyEl.textContent = endToEnd + ' ms';
                    }
                } else {
                    latencyEl.textContent = '--';
                }

                if (data.estimatedFps && data.estimatedFps > 0) {
                    fpsEl.textContent = Number(data.estimatedFps).toFixed(1) + ' FPS';
                } else {
                    fpsEl.textContent = '--';
                }
            } catch (err) {
                document.getElementById('latency').textContent = 'N/A';
                document.getElementById('fps').textContent = 'N/A';
                document.getElementById('clock-hint').textContent = '';
            }
        }

        setInterval(updateStats, 1000);
        setInterval(updateDatasetCaptureStatus, 500);
        updateStats();
        loadDatasetLabels().catch((error) => showCaptureError(error.message));
        updateDatasetCaptureStatus();
    </script>
</body>
</html>
)HTML";

TransmissionStreamServer::TransmissionStreamServer(void)
    : server_sock_fd(-1)
    , server_port(TRANSMISSION_STREAM_DEFAULT_PORT)
    , running(false)
    , server_thread_id(0)
    , latest_frame_id(0)
    , latest_capture_ts_ms(0)
    , ema_fps(0.0)
{
    pthread_mutex_init(&frame_mutex, NULL);
    pthread_cond_init(&frame_cond, NULL);
    pthread_mutex_init(&sock_mutex, NULL);
}

TransmissionStreamServer::~TransmissionStreamServer(void)
{
    stop_server();
    pthread_mutex_destroy(&frame_mutex);
    pthread_cond_destroy(&frame_cond);
    pthread_mutex_destroy(&sock_mutex);
}


/*******************************************************************
 * @brief       获取本机IP地址
 * 
 * @return      返回本机IP地址字符串
 * 
 * @note        自动选择优先级最高的网络接口IP
 ******************************************************************/
std::string TransmissionStreamServer::get_local_ip(void)
{
    struct ifaddrs *ifaddr, *ifa;
    char host[NI_MAXHOST];
    
    if (getifaddrs(&ifaddr) == -1) {
        return "127.0.0.1";
    }
    
    std::string result = "127.0.0.1";
    std::string fallback_ip = "";
    int best_priority = -1;
    
    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == NULL) continue;
        
        int family = ifa->ifa_addr->sa_family;
        
        // 只处理IPv4地址
        if (family == AF_INET) {
            // 跳过回环地址
            if (strcmp(ifa->ifa_name, "lo") == 0) continue;
            
            int s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in),
                              host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
            if (s != 0) continue;
            
            // 计算接口优先级
            int priority = 0;
            bool is_up = (ifa->ifa_flags & IFF_UP) != 0;
            bool is_running = (ifa->ifa_flags & IFF_RUNNING) != 0;
            
            // UP 且 RUNNING 的接口优先级最高
            if (is_up && is_running) {
                priority = 100;
                // wlan/eth 接口额外加分
                if (strncmp(ifa->ifa_name, "wlan", 4) == 0) priority += 20;
                else if (strncmp(ifa->ifa_name, "eth", 3) == 0) priority += 15;
                else if (strncmp(ifa->ifa_name, "en", 2) == 0) priority += 15; // macOS/BSD
                else priority += 5; // 其他接口
            } 
            // 只有 UP 没有 RUNNING 的接口作为备选
            else if (is_up) {
                priority = 50;
                if (strncmp(ifa->ifa_name, "wlan", 4) == 0) priority += 10;
                else if (strncmp(ifa->ifa_name, "eth", 3) == 0) priority += 8;
                else if (strncmp(ifa->ifa_name, "en", 2) == 0) priority += 8;
            }
            // 其他情况优先级很低
            else {
                priority = 10;
            }
            
            // 选择优先级最高的接口
            if (priority > best_priority) {
                best_priority = priority;
                result = host;
            }
            
            // 保存第一个有效IP作为最终备选
            if (fallback_ip.empty() && strcmp(host, "127.0.0.1") != 0) {
                fallback_ip = host;
            }
        }
    }
    
    freeifaddrs(ifaddr);
    
    // 如果没找到合适的，使用备选IP
    if (result == "127.0.0.1" && !fallback_ip.empty()) {
        result = fallback_ip;
    }
    
    return result;
}

/*******************************************************************
 * @brief       关闭服务器socket
 * 
 * @note        线程安全的关闭操作
 ******************************************************************/
void TransmissionStreamServer::close_server_socket(void)
{
    pthread_mutex_lock(&sock_mutex);
    if (server_sock_fd >= 0) {
        shutdown(server_sock_fd, SHUT_RDWR);
        close(server_sock_fd);
        server_sock_fd = -1;
    }
    pthread_mutex_unlock(&sock_mutex);
}

/*******************************************************************
 * @brief       获取当前时间戳(毫秒)
 * 
 * @return      返回当前时间戳(毫秒)
 ******************************************************************/
uint64_t TransmissionStreamServer::now_ms(void)
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

/*******************************************************************
 * @brief       格式化时间戳
 * 
 * @param       ts_ms           时间戳(毫秒)
 * 
 * @return      返回格式化后的时间字符串
 ******************************************************************/
std::string TransmissionStreamServer::format_timestamp(uint64_t ts_ms)
{
    if (ts_ms == 0) return "--";
    time_t seconds = static_cast<time_t>(ts_ms / 1000);
    int ms = static_cast<int>(ts_ms % 1000);
    struct tm tm_time;
    localtime_r(&seconds, &tm_time);
    char date_buf[64];
    strftime(date_buf, sizeof(date_buf), "%Y-%m-%d %H:%M:%S", &tm_time);
    char buf[80];
    snprintf(buf, sizeof(buf), "%s.%03d", date_buf, ms);
    return std::string(buf);
}

/*******************************************************************
 * @brief       发送HTTP响应
 * 
 * @param       sock            客户端socket
 * @param       content_type    内容类型
 * @param       body            响应体
 * @param       body_len        响应体长度
 ******************************************************************/
void TransmissionStreamServer::send_response_status(int sock, int status_code,
                                                     const char* reason,
                                                     const char* content_type,
                                                     const char* body,
                                                     size_t body_len)
{
    std::ostringstream header;
    header << "HTTP/1.1 " << status_code << " " << reason << "\r\n";
    header << "Content-Type: " << content_type << "\r\n";
    header << "Content-Length: " << body_len << "\r\n";
    header << "Cache-Control: no-store\r\n";
    header << "Connection: close\r\n\r\n";
    std::string h = header.str();
    send(sock, h.c_str(), h.length(), MSG_NOSIGNAL);
    send(sock, body, body_len, MSG_NOSIGNAL);
}

void TransmissionStreamServer::send_response(int sock, const char* content_type,
                                             const char* body, size_t body_len)
{
    send_response_status(sock, 200, "OK", content_type, body, body_len);
}

/*******************************************************************
 * @brief       发送统计信息响应
 * 
 * @param       sock            客户端socket
 ******************************************************************/
void TransmissionStreamServer::send_stats_response(int sock)
{
    uint64_t capture_ts = latest_capture_ts_ms;
    uint64_t frame_id = 0;
    pthread_mutex_lock(&frame_mutex);
    frame_id = latest_frame_id;
    pthread_mutex_unlock(&frame_mutex);

    uint64_t server_ts = now_ms();
    double fps = ema_fps;
    std::ostringstream body;
    body << std::fixed << std::setprecision(2)
         << "{\"latestFrameId\":" << frame_id
         << ",\"latestCaptureTsMs\":" << capture_ts
         << ",\"serverTsMs\":" << server_ts
         << ",\"estimatedFps\":" << fps << "}";
    std::string json = body.str();
    send_response(sock, "application/json; charset=utf-8", json.c_str(), json.size());
}

/*******************************************************************
 * @brief       发送MJPEG流
 * 
 * @param       sock            客户端socket
 ******************************************************************/
void TransmissionStreamServer::send_mjpeg_stream(int sock)
{
    const char* header = 
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n\r\n";
    send(sock, header, strlen(header), MSG_NOSIGNAL);
    
    uint64_t last_frame_sent = 0;
    std::vector<unsigned char> jpeg_copy;

    while (running) {
        pthread_mutex_lock(&frame_mutex);
        while (running && latest_frame_id == last_frame_sent) {
            pthread_cond_wait(&frame_cond, &frame_mutex);
        }

        if (!running) {
            pthread_mutex_unlock(&frame_mutex);
            break;
        }

        if (current_jpeg.empty()) {
            pthread_mutex_unlock(&frame_mutex);
            continue;
        }

        jpeg_copy = current_jpeg;
        last_frame_sent = latest_frame_id;
        pthread_mutex_unlock(&frame_mutex);
        
        char boundary[256];
        snprintf(boundary, sizeof(boundary),
                "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n",
                jpeg_copy.size());

        static int send_shrink_counter = 0;
        if (++send_shrink_counter >= 120) {
            jpeg_copy.shrink_to_fit();
            send_shrink_counter = 0;
        }
        
        if (send(sock, boundary, strlen(boundary), MSG_NOSIGNAL) < 0) break;
        if (send(sock, jpeg_copy.data(), jpeg_copy.size(), MSG_NOSIGNAL) < 0) break;
        if (send(sock, "\r\n", 2, MSG_NOSIGNAL) < 0) break;
    }
}

/*******************************************************************
 * @brief       处理拍照请求(发送原始高质量图片到客户端)
 * 
 * @param       sock            客户端socket
 * @param       prefix          文件名前缀
 ******************************************************************/
void TransmissionStreamServer::handle_snapshot_request(int sock, const std::string& prefix)
{   
    // 获取 current_jpeg 的数据
    pthread_mutex_lock(&frame_mutex);
    std::vector<unsigned char> jpeg_copy = current_jpeg;
    pthread_mutex_unlock(&frame_mutex);
    
    if (jpeg_copy.empty()) {
        const char* error_html = "<h1>Error</h1><p>没有可用的图像帧</p>";
        send_response(sock, "text/html; charset=utf-8", error_html, strlen(error_html));
        return;
    }

    // 生成文件名（使用自定义前缀）
    time_t now = time(NULL);
    struct tm tm_time;
    localtime_r(&now, &tm_time);
    char filename[256];
    snprintf(filename, sizeof(filename), "%s_%04d%02d%02d_%02d%02d%02d.jpg",
             prefix.c_str(),
             tm_time.tm_year + 1900, tm_time.tm_mon + 1, tm_time.tm_mday,
             tm_time.tm_hour, tm_time.tm_min, tm_time.tm_sec);
    
    // 发送HTTP响应头（直接发送原始 JPEG）
    std::ostringstream header;
    header << "HTTP/1.1 200 OK\r\n";
    header << "Content-Type: image/jpeg\r\n";
    header << "Content-Length: " << jpeg_copy.size() << "\r\n";
    header << "Content-Disposition: attachment; filename=\"" << filename << "\"\r\n";
    header << "Cache-Control: no-cache\r\n";
    header << "Connection: close\r\n\r\n";
    
    std::string h = header.str();
    send(sock, h.c_str(), h.length(), MSG_NOSIGNAL);
    send(sock, jpeg_copy.data(), jpeg_copy.size(), MSG_NOSIGNAL);
    
    std::cout << "✓ 已发送原始JPEG图片到客户端: " << filename 
              << " (大小: " << jpeg_copy.size() / 1024 << " KB, 摄像头原始JPEG)" << std::endl;
}

/*******************************************************************
 * @brief       处理客户端HTTP请求
 * 
 * @param       sock            客户端socket
 ******************************************************************/
void TransmissionStreamServer::handle_client_request(int sock)
{
    char buffer[4096];
    ssize_t n = recv(sock, buffer, sizeof(buffer) - 1, 0);
    if (n <= 0) {
        close(sock);
        return;
    }
    buffer[n] = '\0';
    
    // 解析请求路径
    std::string request(buffer);
    const size_t method_end = request.find(" ");
    if (method_end == std::string::npos) {
        close(sock);
        return;
    }
    const std::string method = request.substr(0, method_end);
    size_t path_start = method_end + 1;
    size_t path_end = request.find(" ", path_start);
    if (path_end == std::string::npos) {
        close(sock);
        return;
    }
    std::string path = request.substr(path_start, path_end - path_start);
    const std::string request_path = RequestPathOnly(path);
    
    if (path == "/" || path.find("/viewer") == 0 || path.find("/?") == 0) {
        // 返回HTML查看器
        send_response(sock, "text/html; charset=utf-8", viewer_html, strlen(viewer_html));
    } else if (path.find("/stream") == 0) {
        // 返回视频流
        send_mjpeg_stream(sock);
    } else if (path.find("/stats") == 0) {
        send_stats_response(sock);
    } else if (request_path == "/dataset-capture/labels") {
        if (method != "GET") {
            const std::string json = "{\"error\":\"method must be GET\"}";
            send_response_status(sock, 405, "Method Not Allowed",
                                 "application/json; charset=utf-8",
                                 json.c_str(), json.size());
        } else {
            const std::string json = dataset_capture::LabelsJson();
            send_response(sock, "application/json; charset=utf-8",
                          json.c_str(), json.size());
        }
    } else if (request_path == "/dataset-capture/status") {
        if (method != "GET") {
            const std::string json = "{\"error\":\"method must be GET\"}";
            send_response_status(sock, 405, "Method Not Allowed",
                                 "application/json; charset=utf-8",
                                 json.c_str(), json.size());
        } else {
            const std::string json = dataset_capture::StatusJson();
            send_response(sock, "application/json; charset=utf-8",
                          json.c_str(), json.size());
        }
    } else if (request_path == "/dataset-capture/start") {
        if (method != "POST") {
            const std::string json = "{\"error\":\"method must be POST\"}";
            send_response_status(sock, 405, "Method Not Allowed",
                                 "application/json; charset=utf-8",
                                 json.c_str(), json.size());
        } else {
            std::string json;
            const int status = dataset_capture::Start(
                QueryParameter(path, "label"), QueryParameter(path, "mode"), json);
            const char *reason = status == 200 ? "OK" :
                                 status == 400 ? "Bad Request" :
                                 status == 409 ? "Conflict" : "Internal Server Error";
            send_response_status(sock, status, reason,
                                 "application/json; charset=utf-8",
                                 json.c_str(), json.size());
        }
    } else if (request_path == "/dataset-capture/cancel") {
        if (method != "POST") {
            const std::string json = "{\"error\":\"method must be POST\"}";
            send_response_status(sock, 405, "Method Not Allowed",
                                 "application/json; charset=utf-8",
                                 json.c_str(), json.size());
        } else {
            std::string json;
            const int status = dataset_capture::Cancel(json);
            send_response_status(sock, status,
                                 status == 200 ? "OK" : "Conflict",
                                 "application/json; charset=utf-8",
                                 json.c_str(), json.size());
        }
    } else if (path.find("/snapshot") == 0) {
        // 解析文件名前缀参数
        std::string prefix = "snapshot";  // 默认前缀
        size_t query_pos = path.find("?prefix=");
        if (query_pos != std::string::npos) {
            size_t prefix_start = query_pos + 8;  // "?prefix=" 长度为8
            size_t prefix_end = path.find("&", prefix_start);
            if (prefix_end == std::string::npos) {
                prefix_end = path.length();
            }
            prefix = path.substr(prefix_start, prefix_end - prefix_start);
            
            // URL解码（简单处理，只处理常见字符）
            size_t pos = 0;
            while ((pos = prefix.find("%20", pos)) != std::string::npos) {
                prefix.replace(pos, 3, " ");
                pos += 1;
            }
            
            // 安全检查：只允许字母、数字、下划线、中划线
            bool valid = true;
            for (char c : prefix) {
                if (!isalnum(c) && c != '_' && c != '-') {
                    valid = false;
                    break;
                }
            }
            if (!valid || prefix.empty()) {
                prefix = "snapshot";
            }
        }
        
        // 处理拍照请求
        handle_snapshot_request(sock, prefix);
    } else {
        // 404
        const char* not_found = "<h1>404 Not Found</h1>";
        send_response_status(sock, 404, "Not Found", "text/html",
                             not_found, strlen(not_found));
    }
    
    close(sock);
}

/*******************************************************************
 * @brief       客户端处理线程上下文结构体
 * 
 * @note        用于在线程间传递服务器实例和客户端socket
 ******************************************************************/
struct ClientThreadContext {
    TransmissionStreamServer* server_instance;
    int client_socket_fd;
};

/*******************************************************************
 * @brief       客户端处理线程函数
 * 
 * @param       arg             ClientThreadContext指针
 * 
 * @return      返回NULL
 * 
 * @note        每个客户端连接创建独立线程处理
 ******************************************************************/
void* TransmissionStreamServer::client_thread_func(void* arg)
{
    ClientThreadContext* context = static_cast<ClientThreadContext*>(arg);
    if (!context) return NULL;
    
    TransmissionStreamServer* server = context->server_instance;
    int sock = context->client_socket_fd;
    delete context;
    
    if (server) {
        server->handle_client_request(sock);
    } else {
        close(sock);
    }
    
    return NULL;
}

/*******************************************************************
 * @brief       服务器线程函数
 * 
 * @param       arg             TransmissionStreamServer实例指针
 * 
 * @return      返回NULL
 ******************************************************************/
void* TransmissionStreamServer::server_thread_func(void* arg)
{
    TransmissionStreamServer* server = static_cast<TransmissionStreamServer*>(arg);
    if (!server) return NULL;
    
    int server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0) {
        std::cerr << "创建socket失败" << std::endl;
        return NULL;
    }

    pthread_mutex_lock(&server->sock_mutex);
    server->server_sock_fd = server_sock;
    pthread_mutex_unlock(&server->sock_mutex);
    
    int opt = 1;
    setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(server->server_port);
    
    if (bind(server_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "绑定端口失败" << std::endl;
        close(server_sock);
        return NULL;
    }
    
    if (listen(server_sock, 10) < 0) {
        std::cerr << "监听失败" << std::endl;
        close(server_sock);
        return NULL;
    }
    
    // 获取本机IP地址
    std::string local_ip = server->get_local_ip();
    
    std::cout << "\n======================================" << std::endl;
    std::cout << "📡 MJPEG摄像头图传服务器启动成功!" << std::endl;
    std::cout << "======================================" << std::endl;
    std::cout << "监听端口: " << server->server_port << std::endl;
    std::cout << "本机IP: " << local_ip << std::endl;
    std::cout << "请在浏览器访问: http://" << local_ip << ":" << server->server_port << std::endl;
    std::cout << "======================================\n" << std::endl;
    
    while (server->running) {
        struct sockaddr_in client_addr;
        socklen_t len = sizeof(client_addr);
        int client_sock = accept(server_sock, (struct sockaddr*)&client_addr, &len);
        
        if (client_sock < 0) {
            if (!server->running) break;
            continue;
        }
        
        // 优化socket选项
        int flag = 1;
        setsockopt(client_sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
        
        // 设置发送缓冲区大小
        int sndbuf = 65536;
        setsockopt(client_sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

        // 创建客户端线程处理上下文，传递当前服务器实例
        pthread_t client_thread_id;
        ClientThreadContext* client_context = new ClientThreadContext{
            .server_instance = server,
            .client_socket_fd = client_sock
        };
        pthread_create(&client_thread_id, NULL, client_thread_func, client_context);
        pthread_detach(client_thread_id);
    }
    
    server->close_server_socket();
    return NULL;
}

/*******************************************************************
 * @brief       启动摄像头图传服务器
 * 
 * @param       port            服务器监听端口(默认8080)
 * 
 * @return      返回启动状态
 * @retval      0               启动成功
 * @retval      -1              启动失败
 * 
 * @example     //启动摄像头图传服务器
 *              if(transmission_server.start_server(8080) < 0) {
 *                  return -1;
 *              }
 * 
 * @note        在后台线程中启动HTTP服务器，支持浏览器访问
 *              访问 http://<开发板IP>:<port> 即可查看实时画面
 ******************************************************************/
int TransmissionStreamServer::start_server(int port)
{
    if (running) {
        std::cout << "服务器已经在运行中" << std::endl;
        return 0;
    }
    
    server_port = port;
    running = true;
    latest_frame_id = 0;
    current_jpeg.clear();

    if (pthread_create(&server_thread_id, NULL, server_thread_func, this) != 0) {
        std::cerr << "创建服务器线程失败" << std::endl;
        running = false;
        return -1;
    }
    pthread_detach(server_thread_id);
    
    std::cout << "摄像头图传服务器启动中..." << std::endl;
    return 0;
}

/*******************************************************************
 * @brief       更新摄像头帧数据
 * 
 * @param       frame           OpenCV Mat格式的图像帧
 * 
 * @example     transmission_server.update_frame_mat(frame);
 * 
 * @note        将最新的摄像头帧推送到服务器，供客户端获取
 *              自动编码为JPEG格式并计算帧率
 ******************************************************************/
void TransmissionStreamServer::update_frame_mat(const cv::Mat& frame)
{
    // 确保Mat数据有效
    if (frame.empty()) {
        std::cerr << "错误: update_frame_mat() 接收到空Mat图像" << std::endl;
        return;
    }

    uint64_t capture_ts_ms = now_ms();
    
    // 计算FPS
    static uint64_t last_capture_ts_local = 0;
    static double local_fps_estimate = 0.0;
    if (last_capture_ts_local != 0) {
        uint64_t delta = capture_ts_ms - last_capture_ts_local;
        if (delta > 0) {
            double instant_fps = 1000.0 / static_cast<double>(delta);
            if (local_fps_estimate <= 0.0) {
                local_fps_estimate = instant_fps;
            } else {
                local_fps_estimate = 0.85 * local_fps_estimate + 0.15 * instant_fps;
            }
            ema_fps = local_fps_estimate;
        }
    }
    last_capture_ts_local = capture_ts_ms;

    // 编码为JPEG（低质量，用于图传）
    std::vector<unsigned char> jpeg_buffer;
    std::vector<int> params;
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(90); // 质量90
    
    if (cv::imencode(".jpg", frame, jpeg_buffer, params)) {
        latest_capture_ts_ms = capture_ts_ms;
        pthread_mutex_lock(&frame_mutex);
        current_jpeg.swap(jpeg_buffer);
        ++latest_frame_id;
        pthread_cond_broadcast(&frame_cond);
        pthread_mutex_unlock(&frame_mutex);
    }
}

/*******************************************************************
 * @brief       更新摄像头帧数据（零拷贝JPEG）
 * 
 * @param       jpeg_data       JPEG原始数据
 * 
 * @example     camera.capture_frame(frame, false);  // 不解码
 *              transmission_server.update_frame_jpeg(camera.jpeg_nowdata);
 * 
 * @note        直接使用JPEG原始数据，跳过编解码步骤
 *              适用于纯图传场景，性能提升50-100倍
 ******************************************************************/
void TransmissionStreamServer::update_frame_jpeg(const std::vector<uchar>& jpeg_data)
{
    // 确保数据非空
    if (jpeg_data.empty()) {
        std::cerr << "错误: update_frame_jpeg() 接收到空数据" << std::endl;
        return;
    }

    uint64_t capture_ts_ms = now_ms();
    
    // 计算FPS
    static uint64_t last_capture_ts_local = 0;
    static double local_fps_estimate = 0.0;
    if (last_capture_ts_local != 0) {
        uint64_t delta = capture_ts_ms - last_capture_ts_local;
        if (delta > 0) {
            double instant_fps = 1000.0 / static_cast<double>(delta);
            if (local_fps_estimate <= 0.0) {
                local_fps_estimate = instant_fps;
            } else {
                local_fps_estimate = 0.85 * local_fps_estimate + 0.15 * instant_fps;
            }
            ema_fps = local_fps_estimate;
        }
    }
    last_capture_ts_local = capture_ts_ms;

    // 直接使用JPEG数据用于图传，无需重新编码
    latest_capture_ts_ms = capture_ts_ms;
    pthread_mutex_lock(&frame_mutex);

    // 如果容量过大，释放多余内存
    if (current_jpeg.capacity() > jpeg_data.size() * 2) {
        std::vector<unsigned char>().swap(current_jpeg);  // 完全释放
    }

    current_jpeg = jpeg_data;  // 直接赋值，零拷贝
    ++latest_frame_id;
    pthread_cond_broadcast(&frame_cond);
    pthread_mutex_unlock(&frame_mutex);
}

/*******************************************************************
 * @brief       停止摄像头图传服务器
 * 
 * @example     transmission_server.stop_server();
 * 
 * @note        停止服务器并释放所有资源
 ******************************************************************/
void TransmissionStreamServer::stop_server(void)
{
    if (!running) return;
    
    std::cout << "正在停止摄像头图传服务器..." << std::endl;
    running = false;
    close_server_socket();
    pthread_cond_broadcast(&frame_cond);
    
    std::cout << "摄像头图传服务器已停止" << std::endl;
}

/*******************************************************************
 * @brief       检查服务器是否正在运行
 * 
 * @return      返回服务器运行状态
 * @retval      true            服务器正在运行
 * @retval      false           服务器已停止
 * 
 * @example     if(transmission_server.is_running()) {
 *                  //服务器正在运行
 *              }
 ******************************************************************/
bool TransmissionStreamServer::is_running(void)
{
    return running;
}
