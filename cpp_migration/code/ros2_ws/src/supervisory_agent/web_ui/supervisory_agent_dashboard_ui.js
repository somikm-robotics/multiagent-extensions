/**
 * DashboardUI
 * Minimal JS — DOM wiring, SSE listener, and fetch calls only.
 * All logic lives in the C++ bridge.
 */
class DashboardUI {
    constructor() {
        this.historyArea    = document.getElementById('history-area');
        this.statusText = document.getElementById('status-text');
        this.scanEnable    = document.getElementById('scan-enable');
        this.scanLocation  = document.getElementById('scan-location');
        this.scanBtn       = document.getElementById('scan-btn');

        this._startStreams();
        this._loadLocations();
        this._bindButtons();
    }

    // SSE — receives topic messages from C++ bridge
    _startStreams() {
        const historySource = new EventSource('/history/stream');
        historySource.onmessage = (e) => {
            const data = JSON.parse(e.data)
            document.getElementById('inspection-approve').disabled = !data['inspection-approve'];
            document.getElementById('inspection-cancel').disabled = !data['inspection-cancel'];
            document.getElementById('analysis-approve').disabled = !data['analysis-approve'];
            document.getElementById('analysis-cancel').disabled = !data['analysis-cancel'];
            document.getElementById('scan-enable').disabled = !data['scan'];
            //document.getElementById('scan-btn').disabled = !data['scan-btn'];
            this.historyArea.value += data.msg + '\n';
            this.historyArea.scrollTop = this.historyArea.scrollHeight;
        };

        const statusSource = new EventSource('/status/stream');
        statusSource.onmessage = (e) => {
            this.statusText.textContent = e.data;
        };
    }

    // Load locations from C++ bridge into dropdown
    _loadLocations() {
        fetch('/mines/locations')
            .then(r => r.json())
            .then(data => {
                (Array.isArray(data) ? data : []).forEach(loc => {
                    const opt = document.createElement('option');
                    opt.value = opt.textContent = loc;
                    this.scanLocation.appendChild(opt);
                });
            });
    }

    _bindButtons() {
        document.getElementById('inspection-approve').addEventListener('click', () => 
            this._post('/ui/override/mission', { action: 'approve', label: 'inspection' }));
        document.getElementById('inspection-cancel') .addEventListener('click', () => 
            this._post('/ui/override/mission', { action: 'cancel', label: 'inspection' }));
        document.getElementById('analysis-approve')  .addEventListener('click', () => 
            this._post('/ui/override/mission', { action: 'approve', label: 'analysis' }));
        document.getElementById('analysis-cancel')   .addEventListener('click', () => 
            this._post('/ui/override/mission', { action: 'cancel', label: 'analysis' }));

        //document.getElementById('exit-btn')          .addEventListener('click', () => this._post('/exit'));

        this.scanEnable.addEventListener('change', () => {
            this.scanLocation.disabled = !this.scanEnable.checked;
            this.scanBtn.disabled      = !this.scanEnable.checked;
        });

        this.scanBtn.addEventListener('click', () => {
            if (this.scanLocation.value)
                this._post('/scan/incident', { locationName: this.scanLocation.value }, (data) => {
                    document.getElementById('inspection-approve').disabled = false;
                    document.getElementById('inspection-cancel').disabled = false;        
                });
        });
    }

    _post(endpoint, body, onSuccess) {
        fetch(endpoint, {
            method:  'POST',
            headers: body ? { 'Content-Type': 'application/json' } : {},
            body:    body ? JSON.stringify(body) : null
        })
        .then(r => r.json())
        .then(data => {
            document.getElementById('inspection-approve').disabled = true;
            document.getElementById('inspection-cancel').disabled = true;
            document.getElementById('analysis-approve').disabled = true;
            document.getElementById('analysis-cancel').disabled = true;
            //document.getElementById('scan-btn').disabled = true;
            document.getElementById('scan-enable').disabled = true;
            document.getElementById('scan-enable').checked = false;
            if(onSuccess) onSuccess(data)

            this.historyArea.value += `[${data.success ? '✔' : '✖'}] ${data.message}\n`;
            this.historyArea.scrollTop = this.historyArea.scrollHeight;
        });
    }
}
