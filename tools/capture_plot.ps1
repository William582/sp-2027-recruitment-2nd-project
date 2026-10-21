<#
.SYNOPSIS
    Capture SerialPlot frames from a COM port and dump CSV / PNG.

.DESCRIPTION
    Frame format produced by sp::Plotter:
        0xAA 0xBB | size(1 byte) | float[size]      (little-endian IEEE754)
    Default 10 channels:
        yaw_target, yaw_actual, pitch_target, pitch_actual,
        yaw_torque, pitch_torque, distance_cm,
        fold_angle, fold_torque, motor_status
    Time axis assumes 200 Hz (plot_task period = 5 ms).

.NOTES
    Keep this file pure ASCII: Windows PowerShell 5.1 reads BOM-less UTF-8
    as ANSI and non-ASCII bytes can break the parser.

.EXAMPLE
    $sb = [scriptblock]::Create((Get-Content -Raw .\tools\capture_plot.ps1))
    & $sb -Port COM7 -Seconds 10 -Csv out.csv -Png out.png
#>
param(
  [string]$Port = 'COM7',
  [int]$Seconds = 10,
  [string]$Csv = 'capture.csv',
  [string]$Png = '',
  [int]$Baud = 921600,
  [int]$Channels = 17,
  [double]$SampleHz = 200.0
)

$ErrorActionPreference = 'Stop'

$frameSize = 3 + 4 * $Channels
$chunk = New-Object 'System.Byte[]' 8192

# ------------------------------------------------------------
# capture
# ------------------------------------------------------------
$sp = New-Object System.IO.Ports.SerialPort($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$sp.ReadBufferSize = 65536
$sp.ReadTimeout = 200

try {
  $sp.Open()
}
catch {
  Write-Output ("OPEN_FAILED {0} : {1}" -f $Port, $_.Exception.Message)
  return
}

$buffer = New-Object 'System.Collections.Generic.List[byte]'
$deadline = (Get-Date).AddSeconds($Seconds)

while ((Get-Date) -lt $deadline) {
  try {
    $n = $sp.BytesToRead
    if ($n -gt 0) {
      if ($n -gt $chunk.Length) { $n = $chunk.Length }
      $read = $sp.Read($chunk, 0, $n)
      for ($i = 0; $i -lt $read; $i++) { $buffer.Add($chunk[$i]) }
    }
    else {
      Start-Sleep -Milliseconds 2
    }
  }
  catch {
    Start-Sleep -Milliseconds 2
  }
}

$sp.Close()
$sp.Dispose()

$data = $buffer.ToArray()

# ------------------------------------------------------------
# decode (require the next frame header to confirm the sync)
# ------------------------------------------------------------
$rows = New-Object 'System.Collections.Generic.List[object]'
$i = 0
$skipped = 0
$need = 4 * $Channels

while ($i -le ($data.Length - $frameSize)) {
  $isHeader = ($data[$i] -eq 0xAA) -and ($data[$i + 1] -eq 0xBB) -and ($data[$i + 2] -eq $need)
  if ($isHeader) {
    $vals = New-Object 'single[]' $Channels
    for ($c = 0; $c -lt $Channels; $c++) {
      $vals[$c] = [System.BitConverter]::ToSingle($data, $i + 3 + 4 * $c)
    }

    # 语义校验：这些通道都有确定的取值范围，错位的假帧基本都会被挡掉
    $ok = $true
    for ($c = 0; $c -lt $Channels; $c++) {
      if ([single]::IsNaN($vals[$c]) -or [single]::IsInfinity($vals[$c])) { $ok = $false }
    }
    if ($Channels -ge 14) {
      if (($vals[9] -lt -0.5) -or ($vals[9] -gt 7.5)) { $ok = $false }      # motor_status
      if (($vals[10] -lt -0.5) -or ($vals[10] -gt 63.5)) { $ok = $false }   # can_id_mask
      if (($vals[11] -lt -0.5) -or ($vals[11] -gt 122.5)) { $ok = $false }  # rc_status
      if (($vals[12] -lt -0.5) -or ($vals[12] -gt 7.5)) { $ok = $false }    # fold_state
      if (($vals[13] -lt -0.5) -or ($vals[13] -gt 1100.5)) { $ok = $false } # dbus_len
    }

    if ($ok) {
      $rows.Add($vals)
      $i += $frameSize
      continue
    }
  }
  $i++
  $skipped++
}

Write-Output ("BYTES={0} FRAMES={1} SKIPPED={2}" -f $data.Length, $rows.Count, $skipped)

if ($rows.Count -eq 0) {
  Write-Output "NO_FRAME: check USART1 wiring, 921600 baud, and that SerialPlot released the port"
  return
}

# ------------------------------------------------------------
# CSV
# ------------------------------------------------------------
$headers = @(
  'yaw_target', 'yaw_actual', 'pitch_target', 'pitch_actual',
  'yaw_torque', 'pitch_torque', 'distance',
  'fold_angle', 'fold_torque', 'motor_status', 'can_id_mask',
  'rc_status', 'fold_state', 'dbus_len', 'yaw_speed', 'pitch_speed', 'imu_pitch'
)

$inv = [System.Globalization.CultureInfo]::InvariantCulture
$sb = New-Object System.Text.StringBuilder
[void]$sb.AppendLine('index,time_s,' + (($headers[0..($Channels - 1)]) -join ','))

for ($r = 0; $r -lt $rows.Count; $r++) {
  $t = ($r / $SampleHz).ToString('F4', $inv)
  $vals = @()
  for ($c = 0; $c -lt $Channels; $c++) { $vals += $rows[$r][$c].ToString('F6', $inv) }
  [void]$sb.AppendLine(("{0},{1}," -f $r, $t) + ($vals -join ','))
}

$csvFull = [System.IO.Path]::GetFullPath($Csv)
[System.IO.File]::WriteAllText($csvFull, $sb.ToString(), (New-Object System.Text.UTF8Encoding($false)))
Write-Output ("CSV={0}" -f $csvFull)

# ------------------------------------------------------------
# per-channel summary
# ------------------------------------------------------------
for ($c = 0; $c -lt $Channels; $c++) {
  $name = if ($c -lt $headers.Count) { $headers[$c] } else { "ch$($c + 1)" }
  $v = @()
  foreach ($row in $rows) { $v += [double]$row[$c] }
  $m = $v | Measure-Object -Minimum -Maximum -Average
  Write-Output ("CH{0,-2} {1,-13} min={2,9:F4} max={3,9:F4} avg={4,9:F4}" -f ($c + 1), $name, $m.Minimum, $m.Maximum, $m.Average)
}

# ------------------------------------------------------------
# PNG
# ------------------------------------------------------------
if ($Png -ne '') {
  try {
    Add-Type -AssemblyName System.Drawing

    $panes = @(
      [pscustomobject]@{ Title = 'yaw (rad)'; A = 0; B = 1 },
      [pscustomobject]@{ Title = 'pitch (rad)'; A = 2; B = 3 }
    )
    if ($Channels -ge 7) { $panes += [pscustomobject]@{ Title = 'distance (cm)'; A = 6; B = -1 } }
    if ($Channels -ge 9) {
      $panes += [pscustomobject]@{ Title = 'fold angle (rad)'; A = 7; B = -1 }
      $panes += [pscustomobject]@{ Title = 'fold torque (Nm)'; A = 8; B = -1 }
    }
    if ($Channels -ge 10) {
      $panes += [pscustomobject]@{ Title = 'motor online bitmask (1 yaw / 2 pitch / 4 fold)'; A = 9; B = -1 }
    }
    if ($Channels -ge 11) {
      $panes += [pscustomobject]@{ Title = 'CAN rx id mask (1:0x201 2:0x202 4:0x203 8:0x204 16:0x205 32:0x206)'; A = 10; B = -1 }
    }
    if ($Channels -ge 12) {
      $panes += [pscustomobject]@{ Title = 'RC status (0=none, 1xx=open, x=sw_r*10+sw_l; 0=DOWN 1=MID 2=UP)'; A = 11; B = -1 }
    }
    if ($Channels -ge 13) {
      $panes += [pscustomobject]@{ Title = 'fold state (0=DIS 1=HOMING 2=UNFOLDING 3=WAITING 4=FOLDING 5=FOLDED 6=FAULT 7=STOWING)'; A = 12; B = -1 }
    }
    if ($Channels -ge 14) {
      $panes += [pscustomobject]@{ Title = 'DBus len (0 = never received, 1018 = 18 bytes ok)'; A = 13; B = -1 }
    }
    if ($Channels -ge 15) {
      $panes += [pscustomobject]@{ Title = 'yaw speed (rad/s)'; A = 14; B = -1 }
      $panes += [pscustomobject]@{ Title = 'pitch speed (rad/s)'; A = 15; B = -1 }
    }
    if ($Channels -ge 17) {
      $panes += [pscustomobject]@{ Title = 'end IMU pitch (rad, 0 = horizontal)'; A = 16; B = -1 }
    }

    $w = 1280
    $paneH = 240
    if ($panes.Count -gt 3) { $paneH = 165 }
    $h = 40 + $paneH * $panes.Count

    $bmp = New-Object System.Drawing.Bitmap($w, $h)
    $g = [System.Drawing.Graphics]::FromImage($bmp)
    $g.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
    $g.Clear([System.Drawing.Color]::White)

    $font = New-Object System.Drawing.Font('Consolas', 10)
    $axisPen = New-Object System.Drawing.Pen([System.Drawing.Color]::LightGray, 1)
    $refPen = New-Object System.Drawing.Pen([System.Drawing.Color]::Crimson, 2)
    $fbPen = New-Object System.Drawing.Pen([System.Drawing.Color]::RoyalBlue, 2)

    $left = 70
    $right = $w - 20

    foreach ($pane in $panes) {
      $top = 30 + $panes.IndexOf($pane) * $paneH
      $bottom = $top + $paneH - 35

      # robust value range: 1st / 99th percentile, so a single bad frame
      # cannot flatten the whole plot
      $pool = New-Object 'System.Collections.Generic.List[double]'
      foreach ($row in $rows) {
        foreach ($ci in @($pane.A, $pane.B)) {
          if ($ci -lt 0) { continue }
          $pool.Add([double]$row[$ci])
        }
      }
      if ($pool.Count -lt 2) { continue }
      $sorted = $pool.ToArray()
      [array]::Sort($sorted)
      $lo = $sorted[[int][math]::Floor($sorted.Length * 0.01)]
      $hi = $sorted[[int][math]::Floor($sorted.Length * 0.99)]
      if (($hi - $lo) -lt 1e-6) { $hi = $lo + 1e-6 }
      $pad = ($hi - $lo) * 0.15
      $min = $lo - $pad
      $max = $hi + $pad

      $g.DrawRectangle($axisPen, $left, $top, ($right - $left), ($bottom - $top))
      $g.DrawString($pane.Title, $font, [System.Drawing.Brushes]::Black, [single]$left, [single]($top - 20))
      $g.DrawString(('{0:F3}' -f $max), $font, [System.Drawing.Brushes]::DimGray, [single]6, [single]$top)
      $g.DrawString(('{0:F3}' -f $min), $font, [System.Drawing.Brushes]::DimGray, [single]6, [single]($bottom - 14))

      $scaleX = ($right - $left) / [double]($rows.Count - 1)
      $scaleY = ($bottom - $top) / ($max - $min)

      foreach ($ci in @(0, 1)) {
        $ch = $pane.A
        $pen = $refPen
        if ($ci -eq 1) { $ch = $pane.B; $pen = $fbPen }
        if ($ch -lt 0) { continue }

        $pts = New-Object 'System.Collections.Generic.List[System.Drawing.PointF]'
        for ($r = 0; $r -lt $rows.Count; $r++) {
          $v = [double]$rows[$r][$ch]
          if ($v -lt $min) { $v = $min }
          if ($v -gt $max) { $v = $max }
          $x = [single]($left + $r * $scaleX)
          $y = [single]($bottom - ($v - $min) * $scaleY)
          $pts.Add((New-Object System.Drawing.PointF($x, $y)))
        }
        if ($pts.Count -ge 2) { $g.DrawLines($pen, $pts.ToArray()) }
      }
    }

    $g.DrawString('red = target   blue = actual', $font, [System.Drawing.Brushes]::Black, [single]70, [single]6)

    $pngFull = [System.IO.Path]::GetFullPath($Png)
    $bmp.Save($pngFull, [System.Drawing.Imaging.ImageFormat]::Png)
    $g.Dispose()
    $bmp.Dispose()
    Write-Output ("PNG={0}" -f $pngFull)
  }
  catch {
    Write-Output ("PNG_FAILED {0}" -f $_.Exception.Message)
  }
}
