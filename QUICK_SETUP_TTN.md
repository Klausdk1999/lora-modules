# Setup Rápido - The Things Network (TTN)

## 🚀 Passos Rápidos

### 1. Criar Aplicação no TTN (5 minutos)

1. Acesse: https://console.thethingsnetwork.org
2. Faça login ou crie conta
3. Escolha cluster (ex: `nam1` ou `eu1`)
4. **Applications** → **Add Application**
   - Application ID: `river-monitoring` (ou outro nome)
   - Clique **Create Application**
5. **Anote o Application EUI** (aparece na página da aplicação)

### 2. Registrar Gateway (5 minutos)

1. **Gateways** → **Add Gateway**
2. Preencha:
   - **Gateway ID:** (pegue do Wisgate em `https://192.168.0.165` → Status → LoRa)
   - **Frequency Plan:** **AU915**
   - **Gateway EUI:** (pegue do Wisgate)
   - **Coordinates:** Sua localização
3. **Create Gateway**

### 3. Configurar Gateway Wisgate (2 minutos)

1. Acesse: `https://192.168.0.165`
2. **Network → LoRa:**
   - Frequency Plan: **AU915**
3. **Network → LoRa → LoRaWAN:**
   - Server Type: **TTN**
   - Server Address: `nam1.cloud.thethings.network` (ou `eu1` se escolheu EU)
   - Port: `1700`
   - Gateway EUI: (copie do TTN)
4. **Save** e reinicie

### 4. Registrar Dispositivo (2 minutos)

1. Na aplicação → **End Devices** → **Add End Device**
2. Escolha **Manually**
3. Preencha:
   - **DevEUI:** `0011223344556677`
   - **AppEUI:** (cole o Application EUI da aplicação)
   - **AppKey:** `75597d3513647c454cbc8fea8ea9e55a`
   - **Frequency Plan:** **AU915**
   - **LoRaWAN Version:** **1.0.3**
4. **Register End Device**

### 5. Atualizar Código

**Me envie o Application EUI** e eu atualizo o código automaticamente!

Ou você pode editar `heltec-lora32-v2/src/main.cpp` e substituir:
```cpp
static const u1_t PROGMEM APPEUI[8] = { 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // SUBSTITUA AQUI
};
```

**Exemplo:** Se o AppEUI for `0000000000000001`, use:
```cpp
static const u1_t PROGMEM APPEUI[8] = { 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
};
```

### 6. Fazer Upload

Depois de atualizar o AppEUI, faça upload novamente.

---

## ✅ Resumo das Credenciais

- **DevEUI:** `0011223344556677` (já no código)
- **AppEUI:** (você precisa pegar do TTN)
- **AppKey:** `75597d3513647c454cbc8fea8ea9e55a` (já atualizado no código)

---

## 🎯 Próximo Passo

**Me envie o Application EUI do TTN** e eu atualizo o código para você!

