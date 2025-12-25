# Configuração do Servidor LoRaWAN - TTN ou ChirpStack

Este guia explica como configurar o servidor LoRaWAN e registrar seu dispositivo Heltec.

## 📋 Informações do Dispositivo

- **DevEUI:** `00:11:22:33:44:55:66:77` (ou gere um único)
- **AppEUI:** Será gerado quando você criar a aplicação
- **AppKey:** `75597d3513647c454cbc8fea8ea9e55a`
- **Frequency Plan:** AU915

---

## 🌐 Opção 1: The Things Network (TTN) - Recomendado para Iniciantes

### Passo 1: Criar Conta e Aplicação

1. **Acesse:** [console.thethingsnetwork.org](https://console.thethingsnetwork.org)
2. **Crie uma conta** (se ainda não tiver)
3. **Selecione o cluster:** Escolha o mais próximo (ex: `nam1` para América do Norte, `eu1` para Europa)
4. **Crie uma aplicação:**
   - Vá em **Applications** → **Add Application**
   - **Application ID:** Escolha um nome (ex: `river-monitoring`)
   - **Description:** Descrição opcional
   - Clique em **Create Application**

### Passo 2: Registrar o Gateway

1. **Vá em Gateways → Add Gateway**
2. **Preencha:**
   - **Gateway ID:** Use o Gateway ID do seu Wisgate (encontre em `https://192.168.0.165` → Status → LoRa)
   - **Frequency Plan:** **AU915**
   - **Gateway EUI:** Use o Gateway EUI do Wisgate
   - **Coordinates:** Sua localização (opcional)
3. **Salve**

### Passo 3: Configurar Gateway Wisgate

1. **Acesse:** `https://192.168.0.165`
2. **Vá em Network → LoRa:**
   - **Frequency Plan:** **AU915**
3. **Vá em Network → LoRa → LoRaWAN:**
   - **Server Type:** **TTN**
   - **Server Address:** 
     - Para `nam1`: `nam1.cloud.thethings.network`
     - Para `eu1`: `eu1.cloud.thethings.network`
   - **Port:** `1700`
   - **Gateway EUI:** Copie do TTN
4. **Salve e reinicie o gateway**

### Passo 4: Registrar o Dispositivo Heltec

1. **Na aplicação criada, vá em End Devices → Add End Device**
2. **Escolha:** **Manually**
3. **Preencha:**
   - **DevEUI:** `0011223344556677` (sem dois pontos)
   - **AppEUI:** Copie o **Application EUI** da sua aplicação (está na página da aplicação)
   - **AppKey:** `75597d3513647c454cbc8fea8ea9e55a`
   - **Frequency Plan:** **AU915**
   - **LoRaWAN Version:** **1.0.3**
   - **Regional Parameters Revision:** **A** ou **B**
4. **Clique em Register End Device**

### Passo 5: Atualizar Código do Heltec

O código já está atualizado com a AppKey. Você só precisa:
1. Abrir `heltec-lora32-v2/src/main.cpp`
2. Substituir o **AppEUI** com o valor do TTN
3. Fazer upload novamente

---

## 🏢 Opção 2: ChirpStack - Para Servidor Próprio

### Passo 1: Instalar ChirpStack (se ainda não tiver)

**Opção A: Docker (Recomendado)**
```bash
docker run -d --name chirpstack -p 8080:8080 -p 1700:1700/udp chirpstack/chirpstack:latest
```

**Opção B: Instalação Completa**
Siga o guia oficial: [chirpstack.io/docs/](https://www.chirpstack.io/docs/)

### Passo 2: Criar Aplicação no ChirpStack

1. **Acesse:** `http://seu-servidor:8080`
2. **Login:** (crie conta se necessário)
3. **Vá em Applications → Add**
4. **Preencha:**
   - **Name:** `river-monitoring`
   - **Description:** Opcional
5. **Salve e anote o Application ID**

### Passo 3: Criar Device Profile

1. **Vá em Device Profiles → Add**
2. **Preencha:**
   - **Name:** `AU915 Profile`
   - **MAC Version:** `1.0.3`
   - **Regional Parameters Revision:** `A` ou `B`
   - **Frequency Plan:** **AU915**
3. **Salve**

### Passo 4: Registrar Gateway

1. **Vá em Gateways → Add**
2. **Preencha:**
   - **Gateway ID:** Gateway ID do Wisgate
   - **Frequency Plan:** **AU915**
   - **Coordinates:** Sua localização
3. **Salve**

### Passo 5: Configurar Gateway Wisgate

1. **Acesse:** `https://192.168.0.165`
2. **Vá em Network → LoRa:**
   - **Frequency Plan:** **AU915**
3. **Vá em Network → LoRa → LoRaWAN:**
   - **Server Type:** **ChirpStack**
   - **Server Address:** IP do seu servidor ChirpStack
   - **Port:** `1700`
4. **Salve e reinicie**

### Passo 6: Registrar Dispositivo

1. **Na aplicação, vá em Devices → Add**
2. **Preencha:**
   - **Device Name:** `heltec-node-001`
   - **Device EUI:** `0011223344556677`
   - **Application:** Selecione sua aplicação
   - **Device Profile:** Selecione o perfil AU915 criado
   - **Application Key:** `75597d3513647c454cbc8fea8ea9e55a`
3. **Salve**

### Passo 7: Atualizar Código

O código já está atualizado com a AppKey. Você só precisa:
1. Abrir `heltec-lora32-v2/src/main.cpp`
2. Substituir o **AppEUI** com o valor do ChirpStack
3. Fazer upload novamente

---

## 🔑 Conversão de Credenciais

### AppKey fornecida:
```
75597d3513647c454cbc8fea8ea9e55a
```

### Formato para o código (já convertido):
```cpp
static const u1_t PROGMEM APPKEY[16] = { 
  0x75, 0x59, 0x7d, 0x35, 0x13, 0x64, 0x7c, 0x45,
  0x4c, 0xbc, 0x8f, 0xea, 0x8e, 0xa9, 0xe5, 0x5a
};
```

### DevEUI atual:
```
00:11:22:33:44:55:66:77
```

### Formato para o código:
```cpp
static const u1_t PROGMEM DEVEUI[8] = { 
  0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77
};
```

---

## ✅ Checklist

- [ ] Servidor LoRaWAN escolhido (TTN ou ChirpStack)
- [ ] Aplicação criada no servidor
- [ ] Gateway registrado no servidor
- [ ] Gateway Wisgate configurado para AU915
- [ ] Gateway Wisgate conectado ao servidor
- [ ] Dispositivo registrado no servidor com:
  - [ ] DevEUI: `0011223344556677`
  - [ ] AppEUI: (do servidor)
  - [ ] AppKey: `75597d3513647c454cbc8fea8ea9e55a`
- [ ] Código atualizado com AppEUI do servidor
- [ ] Código enviado para o Heltec

---

## 🎯 Qual Escolher?

### The Things Network (TTN):
- ✅ **Gratuito**
- ✅ **Fácil de usar**
- ✅ **Não precisa de servidor próprio**
- ✅ **Ideal para testes e projetos pequenos**
- ❌ Depende de servidor externo

### ChirpStack:
- ✅ **Controle total**
- ✅ **Dados ficam no seu servidor**
- ✅ **Ideal para produção**
- ❌ Precisa de servidor próprio
- ❌ Configuração mais complexa

**Recomendação:** Comece com **TTN** para testes, depois migre para ChirpStack se necessário.

---

## 📞 Próximos Passos

1. Escolha TTN ou ChirpStack
2. Siga os passos acima
3. Me informe o **AppEUI** gerado
4. Atualizarei o código automaticamente
5. Faça upload novamente
6. O dispositivo deve fazer join com sucesso!


