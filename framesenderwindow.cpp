#include "framesenderwindow.h"
#include "ui_framesenderwindow.h"
#include "utility.h"

#include <QFileDialog>
#include <QDebug>
#include "mainwindow.h"

/*
 * notes: need to ensure that you grab pointers when modifying data structures and dont
 * make copies. Also, check # of ms elapsed since last tick since they don't tend to come in at 1ms
 * intervals like we asked.
 * Also, rows default to enabled which is odd because the button state does not reflect that.
*/

FrameSenderWindow::FrameSenderWindow(const QVector<CANFrame> *frames, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::FrameSenderWindow)
{
    ui->setupUi(this);

#ifdef VENDOR_SAPA
	Qt::WindowFlags flags = Qt::Dialog;
	flags |= (Qt::WindowMinimizeButtonHint|Qt::WindowCloseButtonHint);
	setWindowFlags(flags);
#endif
    modelFrames = frames;

    intervalTimer = new QTimer();
    intervalTimer->setInterval(1);

    QStringList headers;
    headers << "En" << "Bus" << "ID" << "Len" << "Data" << "Trigger" << "Modifications" << "Count";
    ui->tableSender->setColumnCount(8);
    ui->tableSender->setColumnWidth(0, 50);
    ui->tableSender->setColumnWidth(1, 50);
    ui->tableSender->setColumnWidth(2, 50);
    ui->tableSender->setColumnWidth(3, 50);
    ui->tableSender->setColumnWidth(4, 220);
    ui->tableSender->setColumnWidth(5, 220);
    ui->tableSender->setColumnWidth(6, 220);
    ui->tableSender->setColumnWidth(7, 80);
    ui->tableSender->setHorizontalHeaderLabels(headers);
    createBlankRow();

    connect(ui->tableSender, SIGNAL(cellChanged(int,int)), this, SLOT(onCellChanged(int,int)));
    connect(intervalTimer, SIGNAL(timeout()), this, SLOT(handleTick()));
    connect(ui->btnClearGrid, SIGNAL(clicked(bool)), this, SLOT(clearGrid()));
    connect(ui->btnDisableAll, SIGNAL(clicked(bool)), this, SLOT(disableAll()));
    connect(ui->btnEnableAll, SIGNAL(clicked(bool)), this, SLOT(enableAll()));
    connect(ui->btnLoadGrid, SIGNAL(clicked(bool)), this, SLOT(loadGrid()));
    connect(ui->btnSaveGrid, SIGNAL(clicked(bool)), this, SLOT(saveGrid()));    
    connect(MainWindow::getReference(), SIGNAL(framesUpdated(int)), this, SLOT(updatedFrames(int)));

    intervalTimer->start();
    elapsedTimer.start();
}

FrameSenderWindow::~FrameSenderWindow()
{
    delete ui;

    intervalTimer->stop();    
    delete intervalTimer;
}

void FrameSenderWindow::createBlankRow()
{
    int row = ui->tableSender->rowCount();
    ui->tableSender->insertRow(row);
#ifndef F_NO_DEBUG
	qDebug() << QString("%1, create blank row = %2").arg(__LINE__).arg(row);
#endif

    QTableWidgetItem *item = new QTableWidgetItem();
    item->setFlags(item->flags() |Qt::ItemIsUserCheckable);
    item->setCheckState(Qt::Unchecked);
    inhibitChanged = true;
    ui->tableSender->setItem(row, 0, item);

    for (int i = 1; i < 8; i++)
    {
        item = new QTableWidgetItem("");
        ui->tableSender->setItem(row, i, item);
    }

    inhibitChanged = false;
}

void FrameSenderWindow::buildFrameCache()
{
    CANFrame thisFrame;
    frameCache.clear();
    for (int i = 0; i < modelFrames->count(); i++)
    {
        thisFrame = modelFrames->at(i);
        if (!frameCache.contains(thisFrame.ID))
        {
            frameCache.insert(thisFrame.ID, thisFrame);
        }
        else
        {
            frameCache[thisFrame.ID] = thisFrame;
        }
    }
}

//remember, negative numbers are special -1 = all frames deleted, -2 = totally new set of frames.
void FrameSenderWindow::updatedFrames(int numFrames)
{
    CANFrame thisFrame;
    if (numFrames == -1) //all frames deleted.
    {
    }
    else if (numFrames == -2) //all new set of frames.
    {
        buildFrameCache();
    }
    else //just got some new frames. See if they are relevant.
    {        
        if (numFrames > modelFrames->count()) return;
        qDebug() << "New frames in sender window";
        //run through the supposedly new frames in order
        for (int i = modelFrames->count() - numFrames; i < modelFrames->count(); i++)
        {
            thisFrame = modelFrames->at(i);
            if (!frameCache.contains(thisFrame.ID))
            {
                frameCache.insert(thisFrame.ID, thisFrame);
            }
            else
            {
                frameCache[thisFrame.ID] = thisFrame;
            }
            processIncomingFrame(&thisFrame);
        }
    }
}

void FrameSenderWindow::processIncomingFrame(CANFrame *frame)
{
    for (int sd = 0; sd < sendingData.count(); sd++)
    {
        if (sendingData[sd].triggers.count() == 0) continue;        
        for (int trig = 0; trig < sendingData[sd].triggers.count(); trig++)
        {
            Trigger *thisTrigger = &sendingData[sd].triggers[trig];
            qDebug() << "Trigger ID: " << thisTrigger->ID;
            qDebug() << "Frame ID: " << frame->ID;
            if (thisTrigger->ID > 0 && thisTrigger->ID == frame->ID)
            {
                if (thisTrigger->bus == frame->bus || thisTrigger->bus == -1)
                {
                    if (thisTrigger->currCount < thisTrigger->maxCount)
                    {
                        if (thisTrigger->milliseconds == 0) //immediate reply
                        {
                            thisTrigger->currCount++;
                            sendingData[sd].count++;
                            doModifiers(sd);
                            updateGridRow(sd);
                            sendCANFrame(&sendingData[sd], sendingData[sd].bus);
                        }
                        else //delayed sending frame
                        {
                            thisTrigger->readyCount = true;
                        }
                    }
                }
            }
        }
    }
}

void FrameSenderWindow::enableAll()
{
    for (int i = 0; i < ui->tableSender->rowCount() - 1; i++)
    {
        ui->tableSender->item(i, 0)->setCheckState(Qt::Checked);
        sendingData[i].enabled = true;
    }
}

void FrameSenderWindow::disableAll()
{
    for (int i = 0; i < ui->tableSender->rowCount() - 1; i++)
    {
        ui->tableSender->item(i, 0)->setCheckState(Qt::Unchecked);
        sendingData[i].enabled = false;
    }
}

void FrameSenderWindow::clearGrid()
{
    if (ui->tableSender->rowCount() == 1) return;
    for (int i = ui->tableSender->rowCount() - 2; i >= 0; i--)
    {
        sendingData[i].enabled = false;
        sendingData.removeAt(i);
        ui->tableSender->removeRow(i);
    }
}

void FrameSenderWindow::saveGrid()
{
    QString filename;
    QFileDialog dialog(this);

    QStringList filters;
    filters.append(QString(tr("Frame Sender Definition (*.fsd)")));

    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setNameFilters(filters);
    dialog.setViewMode(QFileDialog::Detail);
    dialog.setAcceptMode(QFileDialog::AcceptSave);

    if (dialog.exec() == QDialog::Accepted)
    {
        filename = dialog.selectedFiles()[0];

        if (dialog.selectedNameFilter() == filters[0])
        {
            if (!filename.contains('.')) filename += ".fsd";
            saveSenderFile(filename);
        }
    }
}

void FrameSenderWindow::loadGrid()
{
    QString filename;
    QFileDialog dialog(this);

    QStringList filters;
    filters.append(QString(tr("Frame Sender Definition (*.fsd)")));

    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setNameFilters(filters);
    dialog.setViewMode(QFileDialog::Detail);

    if (dialog.exec() == QDialog::Accepted)
    {
        filename = dialog.selectedFiles()[0];

        if (dialog.selectedNameFilter() == filters[0])
        {
            loadSenderFile(filename);
        }
    }
}

void FrameSenderWindow::saveSenderFile(QString filename)
{
    QFile *outFile = new QFile(filename);
    QString outString;

    if (!outFile->open(QIODevice::WriteOnly | QIODevice::Text))
    {
        delete outFile;
        return;
    }

    for (int c = 0; c < sendingData.count(); c++)
    {
        outString.clear();
        if (ui->tableSender->item(c, 0)->checkState() == Qt::Checked)
        {
            outString = "T#";
        }
        else outString = "F#";
        for (int i = 1; i < 7; i++)
        {
            outString.append(ui->tableSender->item(c, i)->text());
            outString.append("#");
        }
        outString.append("\n");
        outFile->write(outString.toUtf8());
    }

    outFile->close();
    delete outFile;

}

void FrameSenderWindow::loadSenderFile(QString filename)
{
    QFile *inFile = new QFile(filename);
    QByteArray line;

    if (!inFile->open(QIODevice::ReadOnly | QIODevice::Text))
    {
        delete inFile;
        return;
    }

#ifdef VENDOR_SAPA	
	ui->tableSender->setRowCount(0);
	ui->tableSender->clearContents(); 
#else
    ui->tableSender->clear();
#endif
    sendingData.clear();

    while (!inFile->atEnd()) {
        line = inFile->readLine().simplified();
        if (line.length() > 2)
        {
            QList<QByteArray> tokens = line.split('#');
            int row = ui->tableSender->rowCount();
#ifdef VENDOR_SAPA	
#ifndef F_NO_DEBUG
			qDebug() << QString("%1, row = %2").arg(__LINE__).arg(row);
#endif
			if (row == 0) 
				createBlankRow();
			else // new blank row will be added in onCellChanged 
				row--;

			ui->tableSender->item(row, 0)->setFlags(
				ui->tableSender->item(row, 0)->flags() |Qt::ItemIsUserCheckable
				);
#else
			// insert row will cause crash issue with null real object
            ui->tableSender->insertRow(row);
			ui->tableSender->item(row, 0)->setFlags(Qt::ItemIsUserCheckable);
#endif
            if (tokens[0] == "T")
            {
                ui->tableSender->item(row, 0)->setCheckState(Qt::Checked);
            }
            else ui->tableSender->item(row, 0)->setCheckState(Qt::Unchecked);
            for (int i = 0; i < 7; i++) ui->tableSender->item(row, i)->setText(tokens[i]);
#ifndef VENDOR_SAPA
			// no need to this since data change will trigger it auto
            for (int k = 0; k < 7; k++) processCellChange(row, k);
#endif

        }
    }
    inFile->close();
    delete inFile;
}

void FrameSenderWindow::onCellChanged(int row, int col)
{
    if (inhibitChanged) return;
    qDebug() << QString("onCellChanged: %1, %2").arg(row).arg(col);
    if (row == ui->tableSender->rowCount() - 1)
    {
        createBlankRow();
    }

    processCellChange(row, col);
}

/// <summary>
/// Called every millisecond to set the system update figures and send frames if necessary.
/// </summary>
/// <param name="sender"></param>
/// <param name="timerEventArgs"></param>
void FrameSenderWindow::handleTick()
{
    FrameSendData *sendData;
    Trigger *trigger;
    int elapsed = elapsedTimer.restart();
    if (elapsed == 0) elapsed = 1;
    //Modifier modifier;
    for (int i = 0; i < sendingData.count(); i++)
    {
        sendData = &sendingData[i];
        if (!sendData->enabled) continue; //abort any processing on this if it is not enabled.
        if (sendData->triggers.count() == 0) return;
        for (int j = 0; j < sendData->triggers.count(); j++)
        {
            trigger = &sendData->triggers[j];
            if (trigger->currCount >= trigger->maxCount) continue; //don't process if we've sent max frames we were supposed to
            if (!trigger->readyCount) continue; //don't tick if not ready to tick
            //is it time to fire?
            trigger->msCounter += elapsed; //gives proper tracking even if timer doesn't fire as fast as it should
            if (trigger->msCounter >= trigger->milliseconds)
            {
                trigger->msCounter = 0;
                sendData->count++;
                trigger->currCount++;
                doModifiers(i);
                updateGridRow(i);
                qDebug() << "About to try to send a frame";
                emit sendCANFrame(&sendingData[i], sendingData[i].bus);
                if (trigger->ID > 0) trigger->readyCount = false; //reset flag if this is a timed ID trigger
            }
        }
    }
}

/// <summary>
/// given an index into the sendingData list we run the modifiers that it has set up
/// </summary>
/// <param name="idx">The index into the sendingData list</param>
void FrameSenderWindow::doModifiers(int idx)
{
    int shadowReg = 0; //shadow register we use to accumulate results
    int first=0, second=0;

    FrameSendData *sendData = &sendingData[idx];
    Modifier *mod;
    ModifierOp op;

    if (sendData->modifiers.count() == 0) return; //if no modifiers just leave right now

    qDebug() << "Executing mods";

    for (int i = 0; i < sendData->modifiers.count(); i++)
    {
        mod = &sendData->modifiers[i];
        for (int j = 0; j < mod->operations.count(); j++)
        {
            op = mod->operations.at(j);
            if (op.first.ID == -1)
            {
                first = shadowReg;
            }
            else first = fetchOperand(idx, op.first);
            second = fetchOperand(idx, op.second);
            switch (op.operation)
            {
            case ADDITION:
                shadowReg = first + second;
                break;
            case AND:
                shadowReg = first & second;
                break;
            case DIVISION:
                shadowReg = first / second;
                break;
            case MULTIPLICATION:
                shadowReg = first * second;
                break;
            case OR:
                shadowReg = first | second;
                break;
            case SUBTRACTION:
                shadowReg = first - second;
                break;
            case XOR:
                shadowReg = first ^ second;
                break;
            case MOD:
                shadowReg = first % second;
            }
        }
        //Finally, drop the result into the proper data byte
        sendData->data[mod->destByte] = (unsigned char) shadowReg;
    }
}

int FrameSenderWindow::fetchOperand(int idx, ModifierOperand op)
{
    CANFrame *tempFrame = NULL;
    if (op.ID == 0) //numeric constant
    {
        if (op.notOper) return ~op.databyte;
        else return op.databyte;
    }
    else if (op.ID == -2) //fetch data from a data byte within the output frame
    {
        if (op.notOper) return ~sendingData.at(idx).data[op.databyte];
        else return sendingData.at(idx).data[op.databyte];
    }
    else //look up external data byte
    {
        tempFrame = lookupFrame(op.ID, op.bus);
        if (tempFrame != NULL)
        {
            if (op.notOper) return ~tempFrame->data[op.databyte];
            else return tempFrame->data[op.databyte];
        }
        else return 0;
    }
}

/// <summary>
/// Try to find the most recent frame given the input criteria
/// </summary>
/// <param name="ID">The ID to find</param>
/// <param name="bus">Which bus to look on (-1 if you don't care)</param>
/// <returns></returns>
CANFrame* FrameSenderWindow::lookupFrame(int ID, int bus)
{
    if (!frameCache.contains(ID)) return NULL;

    if (bus == -1 || frameCache[ID].bus == bus) return &frameCache[ID];

    return NULL;
}

/// <summary>
/// Process a single line from the dataGrid. Right now it seems to not trigger at all after the first adding of the code but that seems to maybe
/// be because whichever field you where just in will show up as nothing to the code.
/// </summary>
/// <param name="line"></param>
void FrameSenderWindow::processModifierText(int line)
{
    qDebug() << "processModifierText";
    QString modString;
    bool firstOp = true;
    bool abort = false;
    QString token;
    ModifierOp thisOp;

    //Example line:
    //d0 = D0 + 1,d1 = id:0x200:d3 + id:0x200:d4 AND 0xF0 - Original version
    //D0=D0+1,D1=ID:0x200:D3+ID:0x200:D4&0xF0
    //This is certainly much harder to parse than the trigger definitions.
    //the left side of the = has to be D0 to D7. After that there is a string of
    //data. Spaces used to be required but no longer are. This makes parsing harder but data entry easier

    //yeah, lots of operations on this one line but it's for a good cause. Removes the convenience English versions of the
    //logical operators and replaces them with the math equivs. Also uppercases and removes all superfluous whitespace
    modString = ui->tableSender->item(line, 6)->text().toUpper().trimmed().replace("AND", "&").replace("XOR", "^").replace("OR", "|").replace(" ", "");
    if (modString != "")
    {
        QStringList mods = modString.split(',');
        sendingData[line].modifiers.clear();
        sendingData[line].modifiers.reserve(mods.length());
        for (int i = 0; i < mods.length(); i++)
        {
            Modifier thisMod;
            thisMod.destByte = 0;
            firstOp = true;

            QString leftSide = Utility::grabAlphaNumeric(mods[i]);
            if (leftSide.startsWith("D") && leftSide.length() == 2)
            {
                thisMod.destByte = leftSide.right(1).toInt();
                thisMod.operations.clear();
            }
            else
            {
                qDebug() << "Something wrong with lefthand val";
                continue;
            }
            if (!(Utility::grabOperation(mods[i]) == "="))
            {
                qDebug() << "Err: No = after lefthand val";
                continue;
            }
            abort = false;

            token = Utility::grabAlphaNumeric(mods[i]);
            if (token[0] == '~')
            {
                thisOp.first.notOper = true;
                token = token.remove(0, 1); //remove the ~ character
            }
            else thisOp.first.notOper = false;
            parseOperandString(token.split(":"), thisOp.first);

            if (mods[i].length() < 2) {
                abort = true;
                thisOp.operation = ADDITION;
                thisOp.second.ID = 0;
                thisOp.second.databyte = 0;
                thisOp.second.notOper = false;
                thisMod.operations.append(thisOp);
            }

            while (!abort)
            {
                QString operation = Utility::grabOperation(mods[i]);
                if (operation == "")
                {
                    abort = true;
                }
                else
                {
                    thisOp.operation = parseOperation(operation);
                    QString secondOp = Utility::grabAlphaNumeric(mods[i]);
                    if (mods[i][0] == '~')
                    {
                        thisOp.second.notOper = true;
                        mods[i] = mods[i].remove(0, 1); //remove the ~ character
                    }
                    else thisOp.second.notOper = false;
                    thisOp.second.bus = sendingData[line].bus;
                    thisOp.second.ID = sendingData[line].ID;
                    parseOperandString(secondOp.split(":"), thisOp.second);
                    thisMod.operations.append(thisOp);
                }

                thisOp.first.ID = -1; //shadow register
                if (mods[i].length() < 2) abort = true;
            }

            sendingData[line].modifiers.append(thisMod);
        }
    }
    //there is no else for the modifiers. We'll accept there not being any
}

void FrameSenderWindow::processTriggerText(int line)
{
    qDebug() << "processTriggerText";
    QString trigger;

    //Example line:
    //id0x200 5ms 10x bus0,1000ms
    //trigger has two levels of syntactic parsing. First you split by comma to get each
    //actual trigger. Then you split by spaces to get the tokens within each trigger
    //trigger = ui->tableSender->item(line, 5)->text().toUpper().trimmed().replace(" ", "");
    trigger = ui->tableSender->item(line, 5)->text().toUpper();
    if (trigger != "")
    {
        QStringList triggers = trigger.split(',');
        sendingData[line].triggers.clear();
        sendingData[line].triggers.reserve(triggers.length());
        for (int k = 0; k < triggers.length(); k++)
        {
            Trigger thisTrigger;
            //start out by setting defaults - should be moved to constructor for class Trigger.
            thisTrigger.bus = -1; //-1 means we don't care which
            thisTrigger.ID = -1; //the rest of these being -1 means nothing has changed it
            thisTrigger.maxCount = -1;
            thisTrigger.milliseconds = -1;
            thisTrigger.currCount = 0;
            thisTrigger.msCounter = 0;
            thisTrigger.readyCount = true;

            QStringList trigToks = triggers[k].split(' ');
            for (int x = 0; x < trigToks.length(); x++)
            {
                QString tok = trigToks.at(x);
                if (tok.left(2) == "ID")
                {
                    thisTrigger.ID = Utility::ParseStringToNum(tok.right(tok.length() - 3));
                    if (thisTrigger.maxCount == -1) thisTrigger.maxCount = 10000000;

                    if (thisTrigger.milliseconds == -1) thisTrigger.milliseconds = 0; //by default don't count, just send it upon trigger
                    thisTrigger.readyCount = false; //won't try counting until trigger hits
                }
                else if (tok.endsWith("MS"))
                {
                    thisTrigger.milliseconds = Utility::ParseStringToNum(tok.left(tok.length()-2));
                    if (thisTrigger.maxCount == -1) thisTrigger.maxCount = 10000000;
                    if (thisTrigger.ID == -1) thisTrigger.ID = 0;
                }
                else if (tok.endsWith("X"))
                {
                    thisTrigger.maxCount = Utility::ParseStringToNum(tok.left(tok.length() - 1));
                    if (thisTrigger.ID == -1) thisTrigger.ID = 0;
                    if (thisTrigger.milliseconds == -1) thisTrigger.milliseconds = 10;
                }
                else if (tok.startsWith("BUS"))
                {
                    thisTrigger.bus = Utility::ParseStringToNum(tok.right(tok.length() - 3));
                }
            }
            //now, find anything that wasn't set and set it to defaults
            if (thisTrigger.maxCount == -1) thisTrigger.maxCount = 10000000;
            if (thisTrigger.milliseconds == -1) thisTrigger.milliseconds = 100;
            if (thisTrigger.ID == -1) thisTrigger.ID = 0;
            sendingData[line].triggers.append(thisTrigger);
        }
    }
    else //setup a default single shot trigger
    {
        Trigger thisTrigger;
        thisTrigger.bus = -1;
        thisTrigger.ID = 0;
        thisTrigger.maxCount = 1;
        thisTrigger.milliseconds = 10;
        sendingData[line].triggers.append(thisTrigger);
    }
}

//Turn a set of tokens into an operand
void FrameSenderWindow::parseOperandString(QStringList tokens, ModifierOperand &operand)
{
    qDebug() << "parseOperandString";
    //example string -> bus:0:id:200:d3

    operand.bus = -1;
    operand.ID = -2;
    operand.databyte = 0;

    for (int i = 0; i < tokens.length(); i++)
    {
        if (tokens[i] == "BUS")
        {
            operand.bus = Utility::ParseStringToNum(tokens[++i]);
        }
        else if (tokens[i] == "ID")
        {
            operand.ID = Utility::ParseStringToNum(tokens[++i]);
        }
        else if (tokens[i].length() == 2 && tokens[i].startsWith("D"))
        {
            operand.databyte = Utility::ParseStringToNum(tokens[i].right(tokens[i].length() - 1));
        }
        else
        {
            operand.databyte = Utility::ParseStringToNum(tokens[i]);
            operand.ID = 0; //special ID to show this is a number not a look up.
        }
    }
}

ModifierOperationType FrameSenderWindow::parseOperation(QString op)
{
    qDebug() << "parseOperation";
    if (op == "+") return ADDITION;
    if (op == "-") return SUBTRACTION;
    if (op == "*") return MULTIPLICATION;
    if (op == "/") return DIVISION;
    if (op == "&") return AND;
    if (op == "|") return OR;
    if (op == "^") return XOR;
    if (op == "%") return MOD;
    return ADDITION;
}

/// <summary>
/// Update the DataGridView with the newest info from sendingData
/// </summary>
/// <param name="idx"></param>
void FrameSenderWindow::updateGridRow(int idx)
{
    qDebug() << "updateGridRow";
    inhibitChanged = true;
    FrameSendData *temp = &sendingData[idx];
    int gridLine = idx;
    QString dataString;
    QTableWidgetItem *item = ui->tableSender->item(gridLine, 7);
    if (item == NULL) item = new QTableWidgetItem();
    item->setText(QString::number(temp->count));
    for (int i = 0; i < temp->len; i++)
    {
        dataString.append(Utility::formatNumber(temp->data[i]));
        dataString.append(" ");
    }
    ui->tableSender->item(gridLine, 4)->setText(dataString);
    inhibitChanged = false;
}

void FrameSenderWindow::processCellChange(int line, int col)
{
    qDebug() << "processCellChange";
    FrameSendData tempData;
    QStringList tokens;
    int tempVal;

#ifndef F_NO_DEBUG
	qDebug() << QString("%1, line = %2, count = %3").arg(__LINE__).arg(line).arg(sendingData.count());
#endif 

    //if this is a new line then create the base object for the line
    if (line >= sendingData.count())
    {
        FrameSendData tempData;
        tempData.enabled = false;
        sendingData.append(tempData);
    }

#ifndef F_NO_DEBUG
	qDebug() << QString("%1, line = %2, count = %3").arg(__LINE__).arg(line).arg(sendingData.count());
#endif 

    sendingData[line].count = 0;
#ifndef F_NO_DEBUG
	qDebug() << QString("%1, line = %2, count = %3").arg(__LINE__).arg(line).arg(sendingData.count());
#endif 

    switch (col)
    {
        case 0: //Enable check box       
            if (ui->tableSender->item(line, 0)->checkState() == Qt::Checked)
            {

#ifdef VENDOR_SAPA            
            	if (sendingData[line].enabled == false) {					
					for (int j = 0; j < sendingData[line].triggers.count(); j++)
        			{
            			Trigger *trigger = &sendingData[line].triggers[j];
            			trigger->currCount = 0;
            			trigger->msCounter = 0;
					}					
            	}
#endif				
				sendingData[line].enabled = true;
            }
            else sendingData[line].enabled = false;
            qDebug() << "Setting enabled to " << sendingData[line].enabled;
            break;
        case 1: //Bus designation
            tempVal = Utility::ParseStringToNum(ui->tableSender->item(line, 1)->text());
            if (tempVal < 0) tempVal = 0;
#ifdef VENDOR_SAPA 
            if (tempVal > 2) tempVal = 1;
#else
            if (tempVal > 1) tempVal = 1;
#endif
            sendingData[line].bus = tempVal;
            qDebug() << "Setting bus to " << tempVal;
            break;
        case 2: //ID field
            tempVal = Utility::ParseStringToNum(ui->tableSender->item(line, 2)->text());
            if (tempVal < 1) tempVal = 1;
            if (tempVal > 0x7FFFFFFF) tempVal = 0x7FFFFFFF;
            sendingData[line].ID = tempVal;
            if (sendingData[line].ID <= 0x7FF) sendingData[line].extended = false;
            else sendingData[line].extended = true;
            qDebug() << "setting ID to " << tempVal;
            break;
        case 3: //length field
            tempVal = Utility::ParseStringToNum(ui->tableSender->item(line, 3)->text());
            if (tempVal < 0) tempVal = 0;
            if (tempVal > 8) tempVal = 8;
            sendingData[line].len = tempVal;
            break;
        case 4: //Data bytes
            for (int i = 0; i < 8; i++) sendingData[line].data[i] = 0;

            tokens = ui->tableSender->item(line, 4)->text().split(" ");
            for (int j = 0; j < tokens.count(); j++)
            {
                sendingData[line].data[j] = (uint8_t)Utility::ParseStringToNum(tokens[j]);
            }
            break;
        case 5: //triggers
            processTriggerText(line);
            break;
        case 6: //modifiers
            processModifierText(line);
            break;
    }
}

#ifdef VENDOR_SAPA
#include <limits>

void FrameSenderWindow::on_btnReArrange_clicked()
{
	ui->tableSender->setVisible(false);
	QRect vporig = ui->tableSender->viewport()->geometry();
	QRect vpnew = vporig;
	vpnew.setWidth(std::numeric_limits<int>::max());
	ui->tableSender->viewport()->setGeometry(vpnew);
	ui->tableSender->resizeColumnsToContents();
	ui->tableSender->resizeRowsToContents();
	ui->tableSender->viewport()->setGeometry(vporig);
	ui->tableSender->setVisible(true);
	
	QTimer::singleShot(0, this, [=](){resize(minimumSizeHint());});
}
#endif
