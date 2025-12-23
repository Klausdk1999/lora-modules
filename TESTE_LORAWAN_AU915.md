# Guia de Teste LoRaWAN - AU915

Este guia explica como testar a comunicação LoRaWAN entre os módulos Heltec e LilyGo com o gateway Wisgate Edge Pro usando a frequência **AU915**.

## 📋 Pré-requisitos

1. **Gateway Wisgate Edge Pro** configurado e conectado à rede
   - IP do gateway: `192.168.0.165`
   - Frequência configurada para **AU915**

2. **Servidor LoRaWAN** configurado (TTN ou ChirpStack)
   - Gateway registrado no servidor
   - Aplicação criada
   - Dispositivos registrados

3. **Hardware:**
   - Heltec WiFi LoRa 32 V2
   - LilyGo LoRa32
   - Cabo USB para programação

---

## 🔧 Passo 1: Configurar o Gateway

1. **Acesse o gateway:**
   - Abra o navegador e vá para: `https://192.168.0.165`
   - Login: `admin` / `admin`

2. **Configure a frequência:**
   - Vá em **Network → LoRa**
   - Defina **Frequency Plan:** **AU915**
   - Salve as configurações

3. **Configure o servidor LoRaWAN:**
   - Vá em **Network → LoRa → LoRaWAN**
   - Configure o servidor (TTN ou ChirpStack)
   - Verifique que o gateway está conectado ao servidor

---

## 📝 Passo 2: Registrar Dispositivos no Servidor LoRaWAN

### Para The Things Network (TTN):

1. **Crie uma aplicação:**
   - Acesse [console.thethingsnetwork.org](https://console.thethingsnetwork.org)
   - Vá em **Applications → Add Application**
   - Anote o **Application EUI** (AppEUI)

2. **Registre cada dispositivo:**
   - Vá em **End Devices → Add End Device**
   - Escolha **Manually**
   - Para **Heltec:**
     - **DevEUI:** `00:11:22:33:44:55:66:77` (ou gere um único)
     - **AppEUI:** Seu Application EUI
     - **AppKey:** TTN gerará automaticamente (copie!)
   - Para **LilyGo:**
     - **DevEUI:** `AA:BB:CC:DD:EE:FF:00:11` (ou gere um único)
     - **AppEUI:** Mesmo Application EUI
     - **AppKey:** TTN gerará automaticamente (copie!)
   - **Frequency Plan:** AU915
   - **LoRaWAN Version:** 1.0.3

### Para ChirpStack:

1. **Crie uma aplicação:**
   - Vá em **Applications → Add**
   - Anote o **Application EUI**

2. **Registre cada dispositivo:**
   - Vá em **Devices → Add**
   - Para **Heltec:**
     - **Device EUI:** `00:11:22:33:44:55:66:77`
     - **Application EUI:** Seu Application EUI
     - **Application Key:** Gere ou insira um
   - Para **LilyGo:**
     - **Device EUI:** `AA:BB:CC:DD:EE:FF:00:11`
     - **Application EUI:** Mesmo Application EUI
     - **Application Key:** Gere ou insira um
   - **Device Profile:** AU915

---

## 💻 Passo 3: Configurar o Código dos Nós

### Para Heltec:

1. **Abra o arquivo:** `heltec-lora32-v2/src/test_lorawan.cpp`

2. **Substitua as credenciais:**
   ```cpp
   static const u1_t PROGMEM APPEUI[8] = { 
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // Seu AppEUI aqui
   };
   
   static const u1_t PROGMEM DEVEUI[8] = { 
     0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77  // Seu DevEUI aqui
   };
   
   static const u1_t PROGMEM APPKEY[16] = { 
     0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,  // Seu AppKey aqui
     0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF
   };
   ```

3. **Converta os valores hex do servidor:**
   - Se o servidor mostra: `00:11:22:33:44:55:66:77`
   - Use no código: `0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77`

### Para LilyGo:

1. **Abra o arquivo:** `lilygo-lora32/src/test_lorawan.cpp`

2. **Substitua as credenciais** (mesmo processo do Heltec)

3. **Ajuste os pinos se necessário:**
   ```cpp
   const lmic_pinmap lmic_pins = {
       .nss = 18,
       .rxtx = LMIC_UNUSED_PIN,
       .rst = 23,      // Mude para 14 se for LoRa32 V1.6
       .dio = {26, 33, 32},
   };
   ```

---

## 🚀 Passo 4: Compilar e Fazer Upload

### Para Heltec:

```bash
cd heltec-lora32-v2
pio run -t upload
pio device monitor
```

### Para LilyGo:

```bash
cd lilygo-lora32
pio run -t upload
pio device monitor
```

---

## 📊 Passo 5: Monitorar a Comunicação

### No Serial Monitor, você verá:

1. **Durante o join:**
   ```
   EV_JOINING - Attempting to join network...
   EV_JOINED - Successfully joined network!
   NetID: [número]
   DevAddr: [endereço]
   ```

2. **Durante transmissão:**
   ```
   Sending: Heltec_Test_1
   Packet counter: 1
   EV_TXSTART - Transmission started
   EV_TXCOMPLETE
   RSSI: -85 dBm
   SNR: 10 dB
   ```

3. **Se houver erro:**
   ```
   EV_JOIN_FAILED - Check your credentials!
   ```

### No Servidor LoRaWAN:

1. **Verifique os pacotes recebidos:**
   - TTN: Vá em **Applications → [sua aplicação] → Live Data**
   - ChirpStack: Vá em **Applications → [sua aplicação] → Device Data**

2. **Verifique o gateway:**
   - Gateway web interface: **Status → LoRa**
   - Deve mostrar contadores de uplink/downlink

---

## ✅ Checklist de Verificação

- [ ] Gateway configurado para **AU915**
- [ ] Gateway conectado ao servidor LoRaWAN
- [ ] Dispositivos registrados no servidor
- [ ] Credenciais (AppEUI, DevEUI, AppKey) atualizadas no código
- [ ] Código compilado e enviado para os nós
- [ ] Serial Monitor mostrando `EV_JOINED`
- [ ] Pacotes sendo recebidos no servidor LoRaWAN
- [ ] Gateway mostrando contadores de uplink

---

## 🐛 Troubleshooting

### Node não faz join (EV_JOIN_FAILED):

1. **Verifique as credenciais:**
   - DevEUI, AppEUI e AppKey devem corresponder **exatamente**
   - Use formato hex correto (0x00, 0x11, etc.)
   - Sem espaços ou caracteres extras

2. **Verifique a frequência:**
   - Gateway: AU915
   - Servidor: AU915
   - Código: `-DCFG_au915=1` no platformio.ini

3. **Verifique o alcance:**
   - Nó deve estar dentro do alcance do gateway
   - Verifique conexões da antena
   - Teste com o nó próximo ao gateway

### Gateway não recebe pacotes:

1. **Verifique status do gateway:**
   - Gateway web: **Status → LoRa**
   - Deve mostrar "Connected" ao servidor

2. **Verifique conexão de rede:**
   - Gateway deve ter acesso à internet
   - Ping o servidor LoRaWAN do gateway

3. **Verifique frequência:**
   - Gateway e nó devem usar a mesma frequência (AU915)

### Servidor não recebe dados:

1. **Verifique gateway no servidor:**
   - Gateway deve estar registrado e online
   - Verifique logs do gateway no servidor

2. **Verifique aplicação:**
   - Dispositivos devem estar registrados na aplicação correta
   - Verifique se o gateway está associado à aplicação

---

## 📝 Notas Importantes

1. **AU915 vs AS923:**
   - AU915 é usado na Austrália e pode ser usado no Brasil
   - AS923-1 é o padrão oficial para Brasil
   - Certifique-se de que está usando AU915 em todos os lugares

2. **Sub-bands AU915:**
   - O código usa `LMIC_selectSubBand(1)` (channels 8-15)
   - Você pode mudar para sub-band 0 (channels 0-7) se necessário
   - Verifique qual sub-band seu gateway está usando

3. **Data Rate:**
   - O código usa DR_SF7 (Data Rate 0)
   - Para maior alcance, use DR_SF9 ou DR_SF10
   - Para maior velocidade, use DR_SF7 ou DR_SF8

4. **Intervalo de transmissão:**
   - Padrão: 30 segundos
   - Pode ser ajustado em `TX_INTERVAL`
   - Respeite os limites de duty cycle da região

---

## 🔗 Arquivos de Código

- **Heltec:** `heltec-lora32-v2/src/test_lorawan.cpp`
- **LilyGo:** `lilygo-lora32/src/test_lorawan.cpp`
- **Configuração Heltec:** `heltec-lora32-v2/platformio.ini`
- **Configuração LilyGo:** `lilygo-lora32/platformio.ini`

---

## 📞 Próximos Passos

Após confirmar que a comunicação está funcionando:

1. Integre os sensores (TF-Luna, etc.)
2. Adicione processamento de dados
3. Configure integração com banco de dados
4. Implemente downlink para configuração remota

