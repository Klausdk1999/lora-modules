# T-Beam AXP2101 v1.2 - Upload Instructions

## Botões da Placa
- **RST**: Reset do ESP32 (botão de reset físico)
- **PWR**: Power button (liga/desliga - segure 6+ segundos)
- **IO38**: Botão de usuário (GPIO 38) - normalmente não usado para upload

## ⚠️ IMPORTANTE: Desconectar Cabos RX/TX Durante Upload
**Antes de fazer upload, desconecte os cabos RX/TX do sensor TF02-Pro:**
- **GPIO 1 (TX)** - cabo do TF02-Pro RX
- **GPIO 3 (RX)** - cabo do TF02-Pro TX

Esses pinos podem interferir no processo de boot/upload do ESP32. Após o upload bem-sucedido, reconecte os cabos para testar o sensor.

## Método 1: Reset Simples (Tentativa Rápida)
1. **Desconecte os cabos RX/TX do TF02-Pro** (GPIO 1 e 3)
2. Certifique-se que a placa está ligada (LED deve estar aceso)
3. Feche o monitor serial se estiver aberto
4. Pressione o botão **RST** brevemente
5. Imediatamente execute o upload:
   ```bash
   pio run -t upload --upload-port COM7
   ```

## Método 2: Modo Boot Manual (Se Método 1 falhar)
1. **Segure o botão RST** (não solte ainda)
2. **Pressione e segure também o botão IO38** (ou o botão de boot, se existir)
3. **Solte o botão RST** (mantenha IO38 pressionado)
4. **Solte o IO38** após 1 segundo
5. Imediatamente execute o upload:
   ```bash
   pio run -t upload --upload-port COM7
   ```

## Método 3: Timing Manual Durante Upload
1. Execute o comando de upload:
   ```bash
   pio run -t upload --upload-port COM7
   ```
2. Quando aparecer "Connecting..." na tela, **pressione e solte rapidamente o botão RST**
3. O upload deve continuar automaticamente

## Verificar Status da Placa
Após upload bem-sucedido, abra o monitor serial:
```bash
pio device monitor --port COM7 --baud 115200
```

Você deve ver:
- Mensagens de inicialização do ESP32
- "Initializing TF02-Pro LiDAR sensor..."
- Leituras do sensor (em modo de teste)

## Troubleshooting

### Placa não liga
- Pressione e segure **PWR** por 6+ segundos
- Verifique bateria ou cabo USB

### Upload continua falhando
- **IMPORTANTE: Desconecte os cabos RX/TX do sensor (GPIO 1 e 3) antes do upload**
- Tente outro cabo USB (use cabo de dados, não só carregamento)
- Tente outra porta USB no computador
- Verifique drivers CH9102 instalados
- Reduza velocidade de upload no `platformio.ini`:
  ```ini
  upload_speed = 115200
  ```

### "Port busy" error
- Feche todos os programas usando COM7 (monitor serial, Arduino IDE, etc.)
- Execute no PowerShell:
  ```powershell
  Get-Process | Where-Object {$_.ProcessName -like "*python*"} | Stop-Process -Force
  ```

## Nota sobre GPIO 38
Na T-Beam v1.2, o GPIO 38 pode ser usado como botão de boot alternativo. Se o método padrão (RST apenas) não funcionar, tente usar RST + IO38 simultaneamente conforme Método 2.
