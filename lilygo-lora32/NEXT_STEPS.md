# Próximos Passos - Testando o TF02-Pro

## ✅ Upload Concluído
O código foi enviado com sucesso para a T-Beam!

## 🔌 Reconectar o Sensor TF02-Pro

1. **Desconecte o cabo USB** (ou mantenha conectado se quiser monitorar via serial)

2. **Reconecte os cabos RX/TX:**
   - **TF02-Pro TX** → **GPIO 3** (RX do ESP32)
   - **TF02-Pro RX** → **GPIO 1** (TX do ESP32)
   - **TF02-Pro VCC** → **5V** (ou 3.3V se suportado)
   - **TF02-Pro GND** → **GND**

3. **Certifique-se que o TF02-Pro está ligado:**
   - LED do sensor deve estar aceso
   - Verifique a alimentação (5V ou 3.3V)

## 📊 Monitorar o Serial

Agora você pode abrir o monitor serial para ver as leituras do sensor:

```bash
cd "C:\Users\Klaus\Documents\Mestrado\LoRa-River-Monitoring\lilygo-lora32"
pio device monitor --port COM7 --baud 115200
```

## 🔍 O que Esperar no Serial

Se o sensor estiver funcionando corretamente, você deve ver:

```
Initializing TF02-Pro LiDAR sensor...
  Configuring Serial2: RX=GPIO 3, TX=GPIO 1
  Waiting for sensor to start sending data...
✓ TF02-Pro sensor initialized successfully
✓ Test reading: Distance = XXX.XX cm, Temperature = XX.X °C, Signal = XXX

[SENSOR TEST MODE - LoRaWAN disabled]
Reading sensor continuously...

Reading #1:
  Distance: XXX.XX cm
  Temperature: XX.X °C
  Signal: XXX
  Valid: Yes/No
  Timestamp: [timestamp]

Reading #2:
...
```

## ❌ Problemas Possíveis

### Sensor não inicializa ("sensor not initialized")
- **Verifique os cabos RX/TX**: Pode estar invertido
  - Tente trocar: TF02-Pro TX ↔ RX
- **Verifique a alimentação**: TF02-Pro precisa de 5V (ou 3.3V)
- **Verifique se o LED do sensor está aceso**
- **Aguarde alguns segundos**: O sensor pode precisar de tempo para inicializar

### Nenhuma saída no serial
- Verifique se a porta está correta (COM7)
- Verifique se o baud rate está correto (115200)
- Pressione o botão RST para reiniciar a placa

### Leituras inválidas ou fora do range
- Verifique se há objetos na frente do sensor (mínimo ~10cm)
- Certifique-se que o sensor está apontado para uma superfície refletora
- O TF02-Pro funciona melhor em distâncias de 10cm a 22m

## 🎯 Quando o Sensor Estiver Funcionando

Após confirmar que o sensor está lendo corretamente:

1. **Teste com diferentes distâncias** (10cm, 50cm, 1m, 5m)
2. **Verifique a precisão das leituras**
3. **Aguarde alguns minutos** para ver várias leituras e confirmar estabilidade

## 🔄 Habilitar LoRaWAN (Opcional)

Quando estiver pronto para testar com LoRaWAN (requer gateway online):

1. Edite `src/main.cpp`
2. Mude `SENSOR_TEST_MODE` de `true` para `false`:
   ```cpp
   #define SENSOR_TEST_MODE        false   // Enable LoRaWAN operation
   ```
3. Faça upload novamente (lembre-se de desconectar RX/TX antes!)

## 📝 Notas

- **Pinos TX/RX**: GPIO 1 e 3 estão corretos para T-Beam AXP2101 v1.2
- **Velocidade Serial**: 115200 baud (configurado para TF02-Pro)
- **Modo de Teste**: Atualmente `SENSOR_TEST_MODE = true` (LoRaWAN desabilitado)
- **Warmup Time**: 500ms (sensor precisa de tempo para inicializar)
