# DetectionSystem By Anurak Ch. [Electronic Engineering IX]
## Hardware
##### WISE-4012 I/O control protocol WiFi Server, Communicated with HTTP Method
##### Industrial camera from MindVision [ Gig-Ethernet, USB ]
## Attention !!!!
#### หากต้องการดูตัวอย่างของ GUI เบื้องต้นสามารถเปิดดูได้ ( gui.py, gui_exposure.py ) แต่ไม่สามารถรันโปรแกรมได้เนื่องจาก ในส่วนของโค๊ดมีการเช็คการเชื่อมต่อของกล้อง โดยกล้องที่เชื่อมต่อจะต้องมีไม่น้อยกว่า 2 ตัวและต้องเป็นกล้องที่เชื่อมต่อด้วย Gig-Ethernet หรือ USB2.0/3.0 ของ MindVision เท่านั้น
## VERSION UPDATE
### - Version 1.1 Prototype
<br>
ตั้งค่าหน่วงเวลาสำหรับการเปิด-ปิดอุปกรณ์ปลายทาง
<p>
   <img width="850" src="https://raw.githubusercontent.com/oatanurakch/DetectionSystem_AnurakCH/main/ExampleProgram/286835912_356495526590107_1043970924255836697_n.png">
</p>
แจ้งเตือนกล้องไม่ครบ 2 ตัว
<p>
   <img width="850" src="https://raw.githubusercontent.com/oatanurakch/DetectionSystem_AnurakCH/main/ExampleProgram/286824021_964983370808575_626504043563435196_n.png">
</p>
เลือกโมเดลที่ใช้ในการตรวจจับชิ้นงาน
<p>
   <img width="850" src="https://raw.githubusercontent.com/oatanurakch/DetectionSystem_AnurakCH/main/ExampleProgram/287686375_1089949578269032_8884246841937220615_n.png">
</p>

### - Version 1.2 ปรับหน้าตา GUI โดยทำการเพิ่ม Add model และ Delete model ไปไว้ใน QAction และทำการใส่ Icon ในบางส่วน
<p>
   <img width="850" src="https://raw.githubusercontent.com/oatanurakch/DetectionSystem_AnurakCH/main/ExampleProgram/287338156_716302086317443_4667691230631953776_n.png">
</p>

### - Version 3.0 Edit Program, Capture by signal ( Rising Edge Mode ) from I/O Module, Compare bit address from I/O Module and bit address from config
<p>
   <img width="850" src="https://raw.githubusercontent.com/oatanurakch/DetectionSystem_AnurakCH/main/ExampleProgram/image.png">
</p>


## Ability of Program
- Add model จากการ Train ด้วย Yolov5 algorithm โดย add ตามขั้นตอนใน Document ( หากทำ Document แล้วจะแจ้ง ) รวมไปถึงการลบ model ที่ไม่ได้ใช้งาน <br>
- การตี label นั้นไม่จำเป็นต้องบอกคำตอบว่าเป็นอะไรแต่ให้ใช้ X1 - X10 โดยสามารถทำ key มา match กับคำตอบในรูปแบบของ JSON <br>
- ในกรณีที่ใช้กล้องไม่เกิน 2 ตัวในการตรวจจับชิ้นงาน ระบบด้านในสามารถที่จะนำคำตอบจากทั้ง 2 กล้องมาเชื่อมต่อหากันเพื่อตัดสินใจผลลัพธ์สิ่งที่ตรวจจับเจอนั้นถูกต้องตามที่กำหนดใน JSON หรือไม่ <br>
- สามารถตั้งค่า % ความถูกต้องเพื่อนำไปตัดสินใจในการตีกรอบในการตรวจจับ เช่น ถ้าระบบตรวจจับมาได้ % ความถูกต้องไม่ถึง 80 % ตามที่ตั้งค่าไว้ในโปรแกรมจะถือว่าชิ้นงานนั้นผิดพลาดทันที <br>
- สามารถปรับค่า Exposure ของกล้องได้ตั้งแต่ 0 - 100 <br>
- สามารถควบคุมอุปกรณ์ปลายทางได้ โดยสามารถปรับตั้งค่าให้เลือกช่อง Output ใช้งานได้ตาม Product ของ I/O Module ที่ใช้โดยเป็นการเชื่อมต่อแบบ TCP/IP ในวง Lan เดียวกัน สามารถตั้งค่าว่าต้องการใช้งานช่อง output ช่องไหนบ้างในกรณีที่ตรวจจับชิ้นงานผิดหรือถูกตามที่กำหนด <br>
- สามารถเรียกเปิดโฟลเดอร์ที่ใช้สำหรับจัดเก็บรูปภาพที่ได้จากการตรวจจับ โดยจะบันทึกเฉพาะไฟล์ที่มีการตรวจจับเจอสิ่งใดสิ่งหนึ่งในภาพนั้น ๆ หากตรวจจับแล้วไม่เจอผลลัพธ์ที่ตรงคำตอบใด ๆ จะลบรูปภาพทิ้ง รูปภาพจะเก็บในลักษณะชื่อไฟล์ ลำดับภาพ-ชื่อโมเดล-กล้องตัวบนหรือล่าง_วัน-เดือน-ปี_ชั่วโมง-นาที-วินาที.jpg โดยเก็บไว้ในโฟลเดอร์ _imagedetection ในโฟลเดอร์ของแต่ละโมเดล <br>
- สามารถสั่งลบไฟล์รูปภาพที่บันทึกไว้ทั้งหมดในกรณีที่พื้นที่จัดเก็บเต็ม โดยไม่ต้องเปิดเข้าไปที่โฟลเดอร์แต่สั่งลบผ่าน GUI ได้เลย <br>
- ในเบื้องต้น model ที่แอดไว้ในระบบรวมไปถึงค่า setting ต่าง ๆ ถูกเขียนไปไฟล์ JSON เพื่อทำการ Load config ที่เคยตั้งไว้มาใช้งานในบางส่วน ปัจจุบันมีการบันทึกค่าความสว่างของกล้อง, เปอร์เซ็นความถูกต้องของการตรวจจับชิ้นงาน, การตั้งค่าต่าง ๆ ของ I/O Module เช่น IP Address, รุ่นของ Module ควบคุมที่ใช้งานในระบบ ไม่ต้องมาปรับตั้งค่าใหม่ทุกครั้ง <br>
- ตัวระบบเบื้องต้นมีการตรวจสอบไฟล์ที่จำเป็นในการใช้งานโดยเน้นหนักไปที่ Folder สำหรับ Model ที่ได้จากการ Train ถ้าไม่ครบตาม requirement จะมี popup แจ้งเตือน และทุก ๆ Error ที่อาจจะเกิดขึ้นจะมี Popup แจ้งเตือนทั้งหมด มีการใช้ try และ except เข้ามาช่วยทำให้ส่วนอื่น ๆ ของโปรแกรมไม่หยุดทำงานแต่จะมี Popup แจ้งเตือนขึ้นมาแทน <br>
- สามารถเลือกใช้งานการถ่ายภาพแบบอัตโนมัติผ่านสัญญาณที่ส่งเข้ามาทาง Input ของ I/O Module โดยสามารถเลือกการใช้งานได้ว่าจะถ่ายภาพเพื่อนำไปตรวจจับแบบ Manual ผ่านการคลิกปุ่ม Capture ใน GUI หรือใช้สัญญาณเป็นตัวสั่งถ่ายภาพ
- โมเดลที่ถูกนำมาใช้ในการตรวจจับจะมีการอ้างอิงถูก Address โดยใช้สัญญาณแบบ 0, 1 จาก input channel นำมาอ่านค่าแล้วแปลงเป็นลักษณะของ Bit address และเทียบกับค่า bit address ที่บันทึกไว้ในแต่ละโมเดล
