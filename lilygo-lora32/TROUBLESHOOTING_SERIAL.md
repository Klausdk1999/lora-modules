# Troubleshooting - Access Denied na Porta COM7

## Problema
Erro: "Access is denied" ou "Permission denied" ao tentar acessar COM7

## Soluções

### 1. Fechar Programas que Usam a Porta Serial
Feche todos os programas que possam estar usando a porta COM7:
- **Arduino IDE** (se estiver aberto)
- **Putty** ou outros terminais seriais
- **Monitor Serial do PlatformIO** (se já estiver aberto em outra janela)
- **Qualquer programa de comunicação serial**

### 2. Parar Processos Python/PlatformIO
Execute no PowerShell:
```powershell
Get-Process | Where-Object {$_.ProcessName -like "*python*" -or $_.ProcessName -like "*pio*"} | Stop-Process -Force
```

### 3. Verificar Gerenciador de Dispositivos
1. Abra o **Gerenciador de Dispositivos** (Win+X → Gerenciador de Dispositivos)
2. Expanda **Portas (COM e LPT)**
3. Procure por **USB-Enhanced-SERIAL CH9102 (COM7)**
4. Se houver um triângulo amarelo ou erro, clique com botão direito → **Desabilitar** → **Habilitar** novamente

### 4. Desconectar e Reconectar o Cabo USB
1. Desconecte o cabo USB da placa
2. Aguarde 5 segundos
3. Reconecte o cabo USB
4. Aguarde o Windows reconhecer o dispositivo
5. Tente abrir o monitor serial novamente

### 5. Usar Outra Porta USB
Se possível, tente conectar a placa em outra porta USB do computador. Isso pode atribuir uma nova porta COM (ex: COM8, COM9).

### 6. Reiniciar o Computador
Se nada funcionar, reinicie o computador. Isso liberará todas as portas seriais.

### 7. Verificar Permissões (Windows)
Se você não tem permissões administrativas, tente executar o PowerShell como Administrador:
1. Clique com botão direito no PowerShell
2. Selecione **Executar como administrador**
3. Execute os comandos novamente

## Comando Alternativo para Monitor Serial

Se o `pio device monitor` não funcionar, você pode tentar:

```bash
# Opção 1: Com filtro
pio device monitor --port COM7 --baud 115200 --filter direct

# Opção 2: Sem filtro
pio device monitor --port COM7 --baud 115200

# Opção 3: Com timeout
pio device monitor --port COM7 --baud 115200 --eol LF
```

## Verificar se a Porta Está Livre

Execute no PowerShell:
```powershell
[System.IO.Ports.SerialPort]::GetPortNames()
```

Isso mostrará todas as portas seriais disponíveis. Se COM7 não aparecer, a placa pode não estar conectada ou reconhecida.

## Verificar Processos Usando a Porta

Execute no PowerShell:
```powershell
netstat -ano | findstr :COM7
```

Se houver algum processo listado, anote o PID e pare o processo:
```powershell
Stop-Process -Id <PID> -Force
```

## Solução Rápida

1. **Feche TODOS os programas** (Arduino IDE, Putty, etc.)
2. **Execute no PowerShell:**
   ```powershell
   Get-Process | Where-Object {$_.ProcessName -like "*python*" -or $_.ProcessName -like "*pio*"} | Stop-Process -Force
   ```
3. **Aguarde 3 segundos**
4. **Tente abrir o monitor serial novamente:**
   ```bash
   pio device monitor --port COM7 --baud 115200
   ```

## Se Nada Funcionar

1. Reinicie o computador
2. Reconecte a placa
3. Tente novamente
