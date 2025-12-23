# Como Usar o Código de Teste LoRaWAN

## 📁 Arquivos Criados

Foram criados arquivos de teste simplificados:
- `heltec-lora32-v2/src/test_lorawan.cpp` - Código de teste para Heltec
- `lilygo-lora32/src/test_lorawan.cpp` - Código de teste para LilyGo

## 🔄 Como Usar

### Opção 1: Renomear Arquivos (Recomendado)

**Para Heltec:**
```bash
cd heltec-lora32-v2/src
# Fazer backup do main.cpp atual
mv main.cpp main_backup.cpp
# Usar o código de teste
mv test_lorawan.cpp main.cpp
```

**Para LilyGo:**
```bash
cd lilygo-lora32/src
# Fazer backup do main.cpp atual
mv main_backup.cpp main_backup.cpp
# Usar o código de teste
mv test_lorawan.cpp main.cpp
```

### Opção 2: Copiar Conteúdo

1. Abra `test_lorawan.cpp`
2. Copie todo o conteúdo
3. Cole em `main.cpp` (substituindo o conteúdo atual)

### Opção 3: Compilar Diretamente (PlatformIO)

No `platformio.ini`, você pode especificar o arquivo fonte:

```ini
[env:heltec_wifi_lora_32_V2]
...
src_filter = +<test_lorawan.cpp>
```

Mas a forma mais simples é renomear os arquivos.

## ✅ Após os Testes

Para voltar ao código original com sensores:

```bash
# Heltec
cd heltec-lora32-v2/src
mv main.cpp test_lorawan.cpp
mv main_backup.cpp main.cpp

# LilyGo
cd lilygo-lora32/src
mv main.cpp test_lorawan.cpp
mv main_backup.cpp main.cpp
```

