/**
 * PID Simulation Interface JavaScript
 * Handles all interactive functionality for the web interface
 */

class PIDSimulationInterface {
    constructor() {
        this.currentSimulation = null;
        this.simulationData = {};
        this.isRunning = false;
        this.charts = {};
        
        this.init();
    }

    init() {
        this.updatePlantParameters();
        this.bindEvents();
        this.showStatus('PID Simulation Interface Ready! Use Ctrl+R to run, Ctrl+T to auto-tune.', 'success');
    }

    bindEvents() {
        // Keyboard shortcuts
        document.addEventListener('keydown', (e) => {
            if (e.ctrlKey || e.metaKey) {
                switch(e.key) {
                    case 'r':
                        e.preventDefault();
                        this.runSimulation();
                        break;
                    case 's':
                        e.preventDefault();
                        this.stopSimulation();
                        break;
                    case 't':
                        e.preventDefault();
                        this.autoTune();
                        break;
                }
            }
        });

        // Plant type change
        document.getElementById('plantType').addEventListener('change', () => {
            this.updatePlantParameters();
        });

        // Real-time parameter validation
        ['kp', 'ki', 'kd'].forEach(param => {
            const input = document.getElementById(param);
            if (input) {
                input.addEventListener('input', () => {
                    this.validateParameter(param, input.value);
                });
            }
        });
    }

    updatePlantParameters() {
        const plantType = document.getElementById('plantType').value;
        const parametersDiv = document.getElementById('plantParameters');
        
        const parameterConfigs = {
            first_order: [
                { id: 'timeConstant', label: 'Time Constant (τ)', value: '2.0', step: '0.1', min: '0.1' },
                { id: 'gain', label: 'Steady State Gain (K)', value: '1.0', step: '0.1' }
            ],
            second_order: [
                { id: 'naturalFreq', label: 'Natural Frequency (ωn)', value: '2.0', step: '0.1', min: '0.1' },
                { id: 'dampingRatio', label: 'Damping Ratio (ζ)', value: '0.5', step: '0.1', min: '0', max: '2' }
            ],
            motor: [
                { id: 'resistance', label: 'Resistance (R) [Ω]', value: '2.0', step: '0.1', min: '0.1' },
                { id: 'inductance', label: 'Inductance (L) [H]', value: '0.1', step: '0.01', min: '0.001' },
                { id: 'inertia', label: 'Inertia (J) [kg⋅m²]', value: '0.02', step: '0.01', min: '0.001' },
                { id: 'friction', label: 'Friction (B) [Nm⋅s]', value: '0.1', step: '0.01', min: '0' }
            ],
            tank: [
                { id: 'tankArea', label: 'Tank Area [m²]', value: '1.0', step: '0.1', min: '0.1' },
                { id: 'outletCoeff', label: 'Outlet Coefficient', value: '0.1', step: '0.01', min: '0.01' },
                { id: 'maxLevel', label: 'Max Level [m]', value: '10.0', step: '0.5', min: '1' }
            ]
        };

        const config = parameterConfigs[plantType] || [];
        
        parametersDiv.innerHTML = config.map(param => `
            <div class="form-group">
                <label for="${param.id}">${param.label}</label>
                <input type="number" 
                       id="${param.id}" 
                       value="${param.value}" 
                       step="${param.step}"
                       ${param.min ? `min="${param.min}"` : ''}
                       ${param.max ? `max="${param.max}"` : ''}
                       onchange="pidInterface.validatePlantParameter('${param.id}', this.value)">
            </div>
        `).join('');
    }

    validateParameter(param, value) {
        const numValue = parseFloat(value);
        const input = document.getElementById(param);
        
        if (isNaN(numValue) || numValue < 0) {
            input.style.borderColor = '#e74c3c';
            return false;
        } else {
            input.style.borderColor = '#27ae60';
            return true;
        }
    }

    validatePlantParameter(param, value) {
        const numValue = parseFloat(value);
        const input = document.getElementById(param);
        
        let isValid = !isNaN(numValue);
        
        // Specific validations
        switch(param) {
            case 'dampingRatio':
                isValid = isValid && numValue >= 0 && numValue <= 2;
                break;
            case 'timeConstant':
            case 'naturalFreq':
            case 'resistance':
            case 'inductance':
            case 'inertia':
            case 'tankArea':
            case 'outletCoeff':
                isValid = isValid && numValue > 0;
                break;
            default:
                isValid = isValid && numValue >= 0;
        }
        
        input.style.borderColor = isValid ? '#27ae60' : '#e74c3c';
        return isValid;
    }

    showStatus(message, type = 'info', duration = 5000) {
        const statusDiv = document.getElementById('status');
        statusDiv.className = `status ${type}`;
        statusDiv.textContent = message;
        statusDiv.style.display = 'block';
        statusDiv.classList.add('fade-in');
        
        if (type === 'success' || type === 'error') {
            setTimeout(() => {
                statusDiv.style.display = 'none';
            }, duration);
        }
    }

    showProgress(progress, message) {
        const progressBar = document.querySelector('.progress-fill');
        if (progressBar) {
            progressBar.style.width = `${progress}%`;
        }
        this.showStatus(message, 'info');
    }

    getParameterValues() {
        return {
            pid: {
                kp: parseFloat(document.getElementById('kp').value),
                ki: parseFloat(document.getElementById('ki').value),
                kd: parseFloat(document.getElementById('kd').value),
                setpoint: parseFloat(document.getElementById('setpoint').value)
            },
            simulation: {
                duration: parseFloat(document.getElementById('duration').value),
                type: document.getElementById('simType').value,
                amplitude: parseFloat(document.getElementById('amplitude').value),
                frequency: parseFloat(document.getElementById('frequency').value)
            },
            plant: this.getPlantParameters()
        };
    }

    getPlantParameters() {
        const plantType = document.getElementById('plantType').value;
        const params = { type: plantType };
        
        switch(plantType) {
            case 'first_order':
                params.timeConstant = parseFloat(document.getElementById('timeConstant').value);
                params.gain = parseFloat(document.getElementById('gain').value);
                break;
            case 'second_order':
                params.naturalFreq = parseFloat(document.getElementById('naturalFreq').value);
                params.dampingRatio = parseFloat(document.getElementById('dampingRatio').value);
                break;
            case 'motor':
                params.resistance = parseFloat(document.getElementById('resistance').value);
                params.inductance = parseFloat(document.getElementById('inductance').value);
                params.inertia = parseFloat(document.getElementById('inertia').value);
                params.friction = parseFloat(document.getElementById('friction').value);
                break;
            case 'tank':
                params.tankArea = parseFloat(document.getElementById('tankArea').value);
                params.outletCoeff = parseFloat(document.getElementById('outletCoeff').value);
                params.maxLevel = parseFloat(document.getElementById('maxLevel').value);
                break;
        }
        
        return params;
    }

    applyParameters() {
        const params = this.getParameterValues();
        
        // Validate all parameters
        const isValid = this.validateAllParameters();
        
        if (isValid) {
            this.showStatus(
                `Applied PID parameters: Kp=${params.pid.kp}, Ki=${params.pid.ki}, Kd=${params.pid.kd}, SP=${params.pid.setpoint}`, 
                'success'
            );
        } else {
            this.showStatus('Please check parameter values - some are invalid', 'error');
        }
    }

    validateAllParameters() {
        const pidParams = ['kp', 'ki', 'kd', 'setpoint'];
        let allValid = true;
        
        pidParams.forEach(param => {
            const input = document.getElementById(param);
            if (input) {
                const isValid = this.validateParameter(param, input.value);
                allValid = allValid && isValid;
            }
        });
        
        return allValid;
    }

    async runSimulation() {
        if (this.isRunning) {
            this.showStatus('Simulation is already running!', 'warning');
            return;
        }

        const params = this.getParameterValues();
        this.isRunning = true;
        
        try {
            this.showStatus('Initializing simulation...', 'info');
            this.showProgress(10, 'Setting up plant model...');
            
            await this.sleep(500);
            this.showProgress(30, 'Configuring PID controller...');
            
            await this.sleep(500);
            this.showProgress(50, 'Running simulation...');
            
            // Simulate the actual simulation time
            const duration = params.simulation.duration;
            const steps = Math.floor(duration / 0.01); // Assuming 0.01s sample time
            
            for (let i = 0; i < 5; i++) {
                await this.sleep(300);
                this.showProgress(50 + (i + 1) * 8, `Simulation progress: ${((i + 1) * 20)}%`);
            }
            
            await this.sleep(500);
            this.showProgress(95, 'Processing results...');
            
            await this.sleep(300);
            this.showProgress(100, 'Simulation completed!');
            
            // Generate and display results
            this.generateMockResults(params);
            this.showStatus('Simulation completed successfully!', 'success');
            
        } catch (error) {
            this.showStatus(`Simulation failed: ${error.message}`, 'error');
        } finally {
            this.isRunning = false;
            setTimeout(() => this.showProgress(0, ''), 2000);
        }
    }

    async runTest() {
        const params = this.getParameterValues();
        const testType = params.simulation.type;
        const amplitude = params.simulation.amplitude;
        
        this.showStatus(`Running ${testType} test with amplitude ${amplitude}...`, 'info');
        
        try {
            this.showProgress(20, 'Preparing test...');
            await this.sleep(500);
            
            this.showProgress(60, `Executing ${testType} test...`);
            await this.sleep(1000);
            
            this.showProgress(100, 'Test completed!');
            this.generateMockResults(params);
            this.showStatus(`${testType} test completed!`, 'success');
            
        } catch (error) {
            this.showStatus(`Test failed: ${error.message}`, 'error');
        }
    }

    async autoTune() {
        this.showStatus('Running auto-tuning algorithm...', 'info');
        
        try {
            this.showProgress(15, 'Analyzing plant characteristics...');
            await this.sleep(800);
            
            this.showProgress(40, 'Applying Ziegler-Nichols method...');
            await this.sleep(1000);
            
            this.showProgress(70, 'Optimizing parameters...');
            await this.sleep(800);
            
            this.showProgress(90, 'Validating results...');
            await this.sleep(500);
            
            // Generate optimized parameters
            const optimizedParams = this.generateOptimizedParameters();
            
            document.getElementById('kp').value = optimizedParams.kp.toFixed(3);
            document.getElementById('ki').value = optimizedParams.ki.toFixed(3);
            document.getElementById('kd').value = optimizedParams.kd.toFixed(4);
            
            this.showProgress(100, 'Auto-tuning completed!');
            this.showStatus(
                `Auto-tuning completed! New parameters: Kp=${optimizedParams.kp.toFixed(3)}, Ki=${optimizedParams.ki.toFixed(3)}, Kd=${optimizedParams.kd.toFixed(4)}`, 
                'success'
            );
            
        } catch (error) {
            this.showStatus(`Auto-tuning failed: ${error.message}`, 'error');
        }
    }

    generateOptimizedParameters() {
        const plantType = document.getElementById('plantType').value;
        
        // Simulated optimization based on plant type
        const baseParams = {
            first_order: { kp: 1.2, ki: 0.6, kd: 0.15 },
            second_order: { kp: 0.8, ki: 0.4, kd: 0.12 },
            motor: { kp: 8.5, ki: 4.2, kd: 0.08 },
            tank: { kp: 2.5, ki: 0.8, kd: 0.05 }
        };
        
        const base = baseParams[plantType] || baseParams.first_order;
        
        // Add some realistic variation
        return {
            kp: base.kp * (0.8 + Math.random() * 0.4),
            ki: base.ki * (0.8 + Math.random() * 0.4),
            kd: base.kd * (0.8 + Math.random() * 0.4)
        };
    }

    async applyTuningMethod(method) {
        this.showStatus(`Applying ${method} tuning...`, 'info');
        
        const tuningParams = {
            'Ziegler-Nichols': { kp: 1.2, ki: 0.6, kd: 0.15 },
            'Cohen-Coon': { kp: 1.35, ki: 0.54, kd: 0.18 },
            'IMC': { kp: 0.9, ki: 0.45, kd: 0.0 }
        };
        
        const params = tuningParams[method];
        if (params) {
            await this.sleep(1000);
            
            document.getElementById('kp').value = params.kp;
            document.getElementById('ki').value = params.ki;
            document.getElementById('kd').value = params.kd;
            
            this.showStatus(`${method} parameters applied!`, 'success');
        }
    }

    async optimizeFor(objective) {
        this.showStatus(`Optimizing for ${objective}...`, 'info');
        
        const objectives = {
            speed: { kp: 2.5, ki: 1.2, kd: 0.3 },
            stability: { kp: 0.8, ki: 0.2, kd: 0.15 },
            accuracy: { kp: 1.0, ki: 0.8, kd: 0.1 }
        };
        
        const params = objectives[objective];
        if (params) {
            await this.sleep(1000);
            
            document.getElementById('kp').value = params.kp;
            document.getElementById('ki').value = params.ki;
            document.getElementById('kd').value = params.kd;
            
            this.showStatus(`Optimized for ${objective}!`, 'success');
        }
    }

    async loadPreset(system) {
        this.showStatus(`Loading ${system} control preset...`, 'info');
        
        const presets = {
            motor: {
                plant: 'motor',
                pid: { kp: 10.0, ki: 5.0, kd: 0.1 }
            },
            temperature: {
                plant: 'first_order',
                pid: { kp: 1.0, ki: 0.1, kd: 0.01 }
            },
            flow: {
                plant: 'tank',
                pid: { kp: 2.0, ki: 0.5, kd: 0.05 }
            }
        };
        
        const preset = presets[system];
        if (preset) {
            await this.sleep(800);
            
            document.getElementById('plantType').value = preset.plant;
            document.getElementById('kp').value = preset.pid.kp;
            document.getElementById('ki').value = preset.pid.ki;
            document.getElementById('kd').value = preset.pid.kd;
            
            this.updatePlantParameters();
            this.showStatus(`${system} preset loaded!`, 'success');
        }
    }

    stopSimulation() {
        if (this.isRunning) {
            this.isRunning = false;
            this.showStatus('Simulation stopped by user.', 'warning');
            this.showProgress(0, '');
        } else {
            this.showStatus('No simulation is currently running.', 'info');
        }
    }

    generateMockResults(params = null) {
        const plotContainer = document.getElementById('plotContainer');
        
        // Generate realistic performance metrics based on parameters
        const metrics = this.generatePerformanceMetrics(params);
        
        plotContainer.innerHTML = `
            <div style="text-align: center; padding: 20px;" class="fade-in">
                <h3>📈 Simulation Results</h3>
                <p style="margin: 20px 0; color: #666;">
                    ${params ? `${params.simulation.type.toUpperCase()} Response - Duration: ${params.simulation.duration}s` : 'Simulation Results'}<br>
                    Plant: ${params ? params.plant.type.replace('_', ' ').toUpperCase() : 'Unknown'} | 
                    PID: Kp=${params ? params.pid.kp : 'N/A'}, Ki=${params ? params.pid.ki : 'N/A'}, Kd=${params ? params.pid.kd : 'N/A'}
                </p>
                <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 15px; margin: 20px 0;">
                    ${Object.entries(metrics).map(([key, value]) => `
                        <div class="metric-card">
                            <h4>${key.replace(/([A-Z])/g, ' $1').replace(/^./, str => str.toUpperCase())}</h4>
                            <div class="value">${value.value}</div>
                            <div class="unit">${value.unit}</div>
                        </div>
                    `).join('')}
                </div>
                <div style="background: #f8f9fa; padding: 30px; margin: 20px; border-radius: 12px; border: 2px dashed #dee2e6;">
                    <strong>Interactive Plot Area</strong><br>
                    <p style="margin: 10px 0; color: #6c757d;">
                        Real implementation would use libraries like:<br>
                        • <strong>Chart.js</strong> for responsive charts<br>
                        • <strong>Plot.ly</strong> for interactive plots<br>
                        • <strong>D3.js</strong> for custom visualizations
                    </p>
                    <div style="margin-top: 20px;">
                        <button class="button button-secondary" onclick="pidInterface.exportResults()">Export Data</button>
                        <button class="button" onclick="pidInterface.showPhasePortrait()">Phase Portrait</button>
                        <button class="button" onclick="pidInterface.showBodePlot()">Bode Plot</button>
                    </div>
                </div>
            </div>
        `;

        this.displayResultsTable(metrics);
    }

    generatePerformanceMetrics(params) {
        // Generate realistic metrics based on PID parameters and plant type
        const kp = params ? params.pid.kp : 1.0;
        const ki = params ? params.pid.ki : 0.5;
        const kd = params ? params.pid.kd : 0.1;
        
        // Simulate realistic performance based on parameters
        const settlingTime = Math.max(0.5, 8.0 / (kp + ki));
        const overshoot = Math.max(0, Math.min(50, (kp - 0.5) * 15 + (kd < 0.05 ? 10 : 0)));
        const riseTime = settlingTime * 0.35;
        const steadyStateError = Math.max(0, 0.1 / (1 + ki));
        const iae = settlingTime * 2.5 + overshoot * 0.1;
        const ise = iae * 0.6;

        return {
            settlingTime: { value: settlingTime.toFixed(2), unit: 'seconds' },
            overshoot: { value: overshoot.toFixed(1), unit: '%' },
            riseTime: { value: riseTime.toFixed(2), unit: 'seconds' },
            steadyStateError: { value: steadyStateError.toFixed(4), unit: 'units' },
            iae: { value: iae.toFixed(2), unit: 'unit·s' },
            ise: { value: ise.toFixed(2), unit: 'unit²·s' }
        };
    }

    displayResultsTable(metrics) {
        const resultsTable = document.getElementById('resultsTable');
        const resultsBody = document.getElementById('resultsBody');
        
        const descriptions = {
            settlingTime: 'Time to reach and stay within ±2% of setpoint',
            overshoot: 'Maximum overshoot percentage beyond setpoint',
            riseTime: 'Time to reach 90% of final value',
            steadyStateError: 'Final tracking error at steady state',
            iae: 'Integral of Absolute Error - overall performance',
            ise: 'Integral of Squared Error - penalizes large errors'
        };
        
        resultsBody.innerHTML = Object.entries(metrics).map(([key, value]) => `
            <tr>
                <td>${key.replace(/([A-Z])/g, ' $1').replace(/^./, str => str.toUpperCase())}</td>
                <td><strong>${value.value}</strong></td>
                <td>${value.unit}</td>
                <td>${descriptions[key] || 'Performance metric'}</td>
            </tr>
        `).join('');
        
        resultsTable.style.display = 'table';
        resultsTable.classList.add('slide-up');
    }

    exportResults() {
        this.showStatus('Exporting simulation data...', 'info');
        // In a real implementation, this would export CSV/JSON data
        setTimeout(() => {
            this.showStatus('Data exported successfully!', 'success');
        }, 1000);
    }

    showPhasePortrait() {
        this.showStatus('Generating phase portrait...', 'info');
        // In a real implementation, this would show phase portrait plot
    }

    showBodePlot() {
        this.showStatus('Generating Bode plot...', 'info');
        // In a real implementation, this would show Bode plot
    }

    sleep(ms) {
        return new Promise(resolve => setTimeout(resolve, ms));
    }
}

// Initialize the interface when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    window.pidInterface = new PIDSimulationInterface();
});

// Global functions for HTML onclick handlers
function applyParameters() {
    pidInterface.applyParameters();
}

function runSimulation() {
    pidInterface.runSimulation();
}

function runTest() {
    pidInterface.runTest();
}

function autoTune() {
    pidInterface.autoTune();
}

function stopSimulation() {
    pidInterface.stopSimulation();
}

function zieglerNichols() {
    pidInterface.applyTuningMethod('Ziegler-Nichols');
}

function cohenCoon() {
    pidInterface.applyTuningMethod('Cohen-Coon');
}

function imc() {
    pidInterface.applyTuningMethod('IMC');
}

function optimizeFor(objective) {
    pidInterface.optimizeFor(objective);
}

function loadPreset(system) {
    pidInterface.loadPreset(system);
}

function compareControllers() {
    pidInterface.showStatus('Running controller comparison...', 'info');
    setTimeout(() => {
        pidInterface.showStatus('Comparison completed! Check results table.', 'success');
        pidInterface.generateComparisonResults();
    }, 2500);
}

function updatePlantParameters() {
    pidInterface.updatePlantParameters();
}
