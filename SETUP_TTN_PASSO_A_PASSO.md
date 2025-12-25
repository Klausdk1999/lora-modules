# Setup TTN - Passo a Passo Completo

## 📋 Checklist de Informações Necessárias

Antes de começar, você precisará:
- [ ] Conta no TTN (vamos criar)
- [ ] Gateway ID do Wisgate
- [ ] Gateway EUI do Wisgate
- [ ] Application EUI (será gerado)

---

## 🚀 PASSO 1: Criar Conta e Aplicação no TTN

### 1.1 Criar Conta
1. Acesse: **https://console.thethingsnetwork.org**
2. Clique em **Create account** (ou faça login se já tiver)
3. Complete o cadastro

### 1.2 Escolher Cluster
1. Após login, escolha um cluster:
   - **nam1** (América do Norte) - Recomendado para Brasil
   - **eu1** (Europa)
   - **au1** (Austrália)
2. Clique no cluster escolhido

### 1.3 Criar Aplicação
1. No menu lateral, clique em **Applications**
2. Clique no botão **+ Add application**
3. Preencha:
   - **Application ID:** `river-monitoring` (ou outro nome único)
   - **Description:** `River Level Monitoring System` (opcional)
   - **Handler registration:** Deixe o padrão
4. Clique em **Create application**

### 1.4 Anotar Application EUI
1. Na página da aplicação, procure por **Application EUI**
2. **COPIE ESSE VALOR** - você precisará dele!
   - Formato: `0000000000000001` (16 caracteres hex)
   - **IMPORTANTE:** Anote sem os dois pontos

---

## 📡 PASSO 2: Obter Informações do Gateway Wisgate

### 2.1 Acessar Gateway
1. Abra o navegador
2. Acesse: **https://192.168.0.165**
3. Login: `admin` / `admin` (ou sua senha)

### 2.2 Encontrar Gateway ID e EUI
1. Vá em **Status → LoRa**
2. Procure por:
   - **Gateway ID:** (ex: `B827EBFFFF123456`)
   - **Gateway EUI:** (ex: `B827EBFFFF123456`)
3. **ANOTE AMBOS OS VALORES**

---

## 🔧 PASSO 3: Registrar Gateway no TTN

### 3.1 Adicionar Gateway
1. No TTN, vá em **Gateways** (menu lateral)
2. Clique em **+ Add gateway**

### 3.2 Preencher Informações
1. **Gateway ID:** Cole o Gateway ID do Wisgate
   - Exemplo: `B827EBFFFF123456`
2. **Frequency Plan:** Selecione **AU915**
3. **Gateway EUI:** Cole o Gateway EUI do Wisgate
   - Exemplo: `B827EBFFFF123456`
4. **Description:** `Wisgate Edge Pro - River Monitoring` (opcional)
5. **Coordinates:** 
   - Clique no mapa ou digite suas coordenadas
   - Latitude e Longitude
6. **Antenna placement:** Escolha (indoor/outdoor)
7. Clique em **Create gateway**

### 3.3 Verificar Status
- O gateway deve aparecer na lista
- Status inicial pode ser "Never seen" (normal até configurar)

---

## ⚙️ PASSO 4: Configurar Gateway Wisgate

### 4.1 Configurar Frequência
1. No Wisgate (`https://192.168.0.165`)
2. Vá em **Network → LoRa**
3. Configure:
   - **Frequency Plan:** **AU915**
4. Clique em **Save**

### 4.2 Configurar Servidor LoRaWAN
1. Vá em **Network → LoRa → LoRaWAN**
2. Configure:
   - **Server Type:** **TTN** (ou **The Things Network**)
   - **Server Address:** 
     - Se escolheu `nam1`: `nam1.cloud.thethings.network`
     - Se escolheu `eu1`: `eu1.cloud.thethings.network`
   - **Port:** `1700`
   - **Gateway EUI:** Cole o Gateway EUI que você anotou
3. Clique em **Save**

### 4.3 Reiniciar Gateway
1. Vá em **System → Reboot**
2. Clique em **Reboot**
3. Aguarde 1-2 minutos para reiniciar

### 4.4 Verificar Conexão
1. Após reiniciar, vá em **Status → LoRa**
2. Verifique se mostra **Connected** ou status similar
3. No TTN, verifique se o gateway aparece como **Connected**

---

## 📱 PASSO 5: Registrar Dispositivo Heltec no TTN

### 5.1 Adicionar End Device
1. No TTN, vá para sua aplicação (`river-monitoring`)
2. Clique em **End devices** (aba superior)
3. Clique em **+ Add end device**

### 5.2 Escolher Método
1. Selecione **Manually** (não use "From the LoRaWAN Device Repository")

### 5.3 Preencher Informações do Dispositivo
1. **Frequency Plan:** **AU915**
2. **LoRaWAN Version:** **1.0.3**
3. **Regional Parameters Revision:** **A** (ou **B**)

### 5.4 Preencher Credenciais
1. **DevEUI:**
   - Digite: `0011223344556677`
   - (sem dois pontos, tudo junto)
2. **AppEUI (Join EUI):**
   - Cole o **Application EUI** que você anotou no Passo 1.4
   - Exemplo: `0000000000000001`
3. **AppKey:**
   - Digite: `75597d3513647c454cbc8fea8ea9e55a`
   - (sem dois pontos, tudo junto)

### 5.5 Finalizar
1. Clique em **Register end device**
2. O dispositivo deve aparecer na lista

---

## 💻 PASSO 6: Atualizar Código do Heltec

### 6.1 Obter Application EUI
- Você já anotou no Passo 1.4
- Formato: `0000000000000001` (16 caracteres hex)

### 6.2 Converter para Formato do Código
Se o Application EUI for `0000000000000001`, converta para:
```cpp
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
```

**Exemplo de conversão:**
- `0000000000000001` → `0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01`
- `1234567890ABCDEF` → `0x12, 0x34, 0x56, 0x78, 0x90, 0xAB, 0xCD, 0xEF`

### 6.3 Atualizar Código
1. Abra: `heltec-lora32-v2/src/main.cpp`
2. Encontre a linha com `APPEUI`
3. Substitua pelos valores convertidos

**Exemplo:**
```cpp
// ANTES:
static const u1_t PROGMEM APPEUI[8] = { 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// DEPOIS (exemplo com 0000000000000001):
static const u1_t PROGMEM APPEUI[8] = { 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
};
```

---

## 🔄 PASSO 7: Fazer Upload do Código Atualizado

### 7.1 Compilar e Enviar
```bash
cd heltec-lora32-v2
platformio run -t upload
```

### 7.2 Monitorar Serial
```bash
platformio device monitor
```

---

## ✅ PASSO 8: Verificar Funcionamento

### 8.1 No Serial Monitor
Você deve ver:
```
EV_JOINING - Attempting to join network...
EV_JOINED - Successfully joined network!
NetID: [número]
DevAddr: [endereço]
Sending: Heltec_Test_1
EV_TXCOMPLETE
RSSI: -85 dBm
SNR: 10 dB
```

### 8.2 No OLED
- Deve mostrar: **"JOINED"**
- Contador de pacotes
- RSSI/SNR

### 8.3 No TTN
1. Vá em sua aplicação → **Live data**
2. Você deve ver os pacotes chegando!
3. Cada pacote mostra:
   - Timestamp
   - Payload (dados)
   - RSSI
   - SNR

---

## 🐛 Troubleshooting

### Gateway não conecta ao TTN
- Verifique o Server Address (deve ser `nam1.cloud.thethings.network` ou `eu1...`)
- Verifique a porta (1700)
- Verifique o Gateway EUI
- Reinicie o gateway

### Dispositivo não faz join
- Verifique se DevEUI, AppEUI e AppKey estão **exatamente** iguais no código e no TTN
- Verifique se o gateway está conectado ao TTN
- Verifique se a frequência é AU915 em todos os lugares
- Aguarde alguns minutos (join pode demorar)

### Não recebe dados no TTN
- Verifique se o gateway está "Connected" no TTN
- Verifique se o dispositivo fez join (deve aparecer "EV_JOINED")
- Verifique a distância entre dispositivo e gateway

---

## 📞 Próximos Passos

Depois que tudo estiver funcionando:
1. Configure integração com banco de dados (opcional)
2. Configure webhooks para enviar dados para seu servidor
3. Adicione mais dispositivos
4. Configure alertas

---

## 🎯 Resumo Rápido

1. ✅ Criar aplicação no TTN → Anotar Application EUI
2. ✅ Pegar Gateway ID e EUI do Wisgate
3. ✅ Registrar gateway no TTN
4. ✅ Configurar Wisgate para TTN
5. ✅ Registrar dispositivo no TTN
6. ✅ Atualizar código com Application EUI
7. ✅ Fazer upload
8. ✅ Verificar funcionamento

**Tempo estimado:** 15-20 minutos


