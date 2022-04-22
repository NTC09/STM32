Dim stt, i As Integer
Dim ngay, thang, nam, gio As Date
Dim tenFile As String
Dim vaoRa As String
Dim ComInput, TX As String

Dim xeMayVao, xeMayRa, OtoVao, OtoRa, tongOto, tongXeMay As Integer

Dim xlApp As Excel.Application
Dim xlWB As Excel.Workbook
Dim xlWS As Excel.Worksheet

Private Sub Command1_Click()
    ngay = Day(Now)
    thang = Month(Now)
    nam = Year(Now)
    tenFile = "C:\MyData\" & ngay & "_" & thang & "_" & nam & ".xlsx"
    Text13.Text = tenFile
    Set xlApp = CreateObject("Excel.Application")

    On Error GoTo TaoFile 'Loi file chua duoc tao
    Set xlWB = xlApp.Workbooks.Open(tenFile)
    Set xlWS = xlWB.Worksheets(1)

    TaoFile:
        If Err.Number = 1004 Then
            Set xlWB = xlApp.Workbooks.Add
            Set xlWS = xlWB.Sheets.Item(1)
            xlWS.Cells(1, 1) = "STT"
            xlWS.Cells(1, 2) = "Ma So"
            xlWS.Cells(1, 3) = "Loai Xe"
            xlWS.Cells(1, 4) = "Bien So"
            xlWS.Cells(1, 5) = "Vao/Ra"
            xlWS.Cells(1, 6) = "Ngay"
            xlWS.Cells(1, 7) = "Gio"
            xlWB.SaveAs (tenFile)
            'Range("C5:C" & 7).ColumnWidth = 25
            'Range("D5:D" & 7).ColumnWidth = 15
            'Screen.MousePointer = vbDefault
        End If

    ngay = Date
    gio = Time

    Do While xlWS.Cells(stt, 1) <> ""
    stt = stt + 1
    Loop
    xlWS.Cells(stt, 1) = stt - 1
    xlWS.Cells(stt, 2) = Text1.Text
    xlWS.Cells(stt, 3) = Text3.Text
    xlWS.Cells(stt, 4) = Text2.Text
    xlWS.Cells(stt, 5) = vaoRa
    xlWS.Cells(stt, 6) = ngay
    xlWS.Cells(stt, 7) = gio
    xlWB.Save
    xlWB.Close
    xlApp.Application.Quit
    MsgBox "Da chen du lieu", vbInformation, "Xong"
    End Sub

Private Sub Command10_Click()
    ComTransmit ("SV3OFF")
    End Sub

Private Sub Command11_Click()
    ComTransmit ("SV4OFF")
    End Sub

Private Sub Command12_Click()
    ComTransmit ("UP")
    End Sub

Private Sub Command2_Click()
    ComTransmit ("SV1ON")
    End Sub

Private Sub Command3_Click()
    ComTransmit ("SV2ON")
    End Sub

Private Sub Command4_Click()
    ComTransmit ("SV3ON")
    End Sub

Private Sub Command5_Click()
    ComTransmit ("SV4ON")
    End Sub

Private Sub Command6_Click()
    TX = "A" & Asc(Text5.Text) & " " & Asc(Text9.Text)
    MSComm1.Output = TX
    End Sub

Private Sub Command7_Click()
    On Error GoTo ComERR
    MSComm1.PortOpen = True
    MsgBox "Ket noi thanh cong", vbInformation, "COM"
    MSComm1.Output = "Hello"
    ComERR:
        If Err.Number = 8002 Then
            MsgBox "Loi cong COM", vbInformation, "COM"
        End If
    End Sub

Private Sub Command8_Click()
    ComTransmit ("SV1OFF")
    End Sub

Private Sub Command9_Click()
    ComTransmit ("SV2OFF")
    End Sub

Private Sub Form_Load()
    i = 0
    stt = 2
    Timer1.Enabled = False
    End Sub

    Private Sub MSComm1_OnComm()
    If MSComm1.CommEvent = 2 Then
        ComInput = MSComm1.Input
        Text13.Text = ComInput
        If Left(ComInput, 2) = "OV" Then
            OtoVao = Val(Right(ComInput, 3))
            Text3.Text = "O to"
            Text4.Text = "Vao"
        ElseIf Left(ComInput, 2) = "OR" Then
            OtoRa = Val(Right(ComInput, 3))
            Text3.Text = "O to"
            Text4.Text = "Ra"
        ElseIf Left(ComInput, 2) = "XV" Then
            xeMayVao = Val(Right(ComInput, 3))
            Text3.Text = "Xe may"
            Text4.Text = "Vao"
        ElseIf Left(ComInput, 2) = "XR" Then
            xeMayRa = Val(Right(ComInput, 3))
            Text3.Text = "Xe may"
            Text4.Text = "Ra"
        End If
        'OV  3
        'ComInput = ""
    End If

    Text6.Text = OtoVao
    Text7.Text = OtoRa
    Text8.Text = OtoVao - OtoRa
    Text10.Text = xeMayVao
    Text11.Text = xeMayRa
    Text12.Text = xeMayVao - xeMayRa
    End Sub

Private Sub Timer1_Timer()
    If i = 0 Then
        Label6.Visible = True
        i = 1
    ElseIf i = 1 Then
        Label6.Visible = False
        i = 0
    End If
    End Sub
Private Sub ComTransmit(TX)
    On Error GoTo ComERR
    MSComm1.Output = TX

    ComERR:
    If Err.Number = 8018 Then
            MsgBox "Loi cong COM", vbInformation, "COM"
        End If
    End Sub
