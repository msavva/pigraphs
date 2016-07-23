using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO;
using System.IO.Pipes;

namespace UIWindow
{
    public partial class UIWindow : Form
    {
        const int WM_NCLBUTTONDBLCLK = 0x00A3;  // double click on a title bar a.k.a. non-client area of the form
        const int kMinifiedHeight = 150;
        const int kFullHeight = 690;
        const string pipeBaseName = "UnderstandingScenes";

        NamedPipeServerStream server;
        NamedPipeClientStream client;
        StreamReader reader;
        StreamWriter writer;

        bool ignoreMessages = false;
        bool ignoreScrollEvent = false;
        bool initializingInteractionSets = false;
        bool isMinified = false;

        ConcurrentQueue<string> messages = new ConcurrentQueue<string>();

        float recDurationInSec = -1;
        float timeStartSec = -1, timeEndSec = -1;
        string scanId;
        string scanDir;
        string recDir;
        Tuple<float, float, string> newAnnGuard = Tuple.Create(-1.0f, -1.0f, "New");
        AnnotatorWindow segmentAnnotationWindow = null;
        Dictionary<string, List<string>> interactionSets = null;
        Dictionary<string, Dictionary<string, List<string>>> interactionSetsForRecording = null;
        
        public UIWindow()
        {
            InitializeComponent();
            //ConsoleRedirector.attach(WriteStdOutputToTextBox, true);
            clearAnnotations();
            createAnnotationWindows();
            modelLoadTypeDropdown.SelectedIndex = 0;
            classifierTypeDropdown.SelectedIndex = 0;
            interactionTypeDropdown.SelectedIndex = 0;
        }

        protected override void WndProc(ref Message m) {
          if (m.Msg == WM_NCLBUTTONDBLCLK) {
            m.Result = IntPtr.Zero;
            if (isMinified) {
              this.Height = kFullHeight;
            } else {
              this.Height = kMinifiedHeight;
            }
            isMinified = !isMinified;
            return;
          }
          base.WndProc(ref m);
        }

        private void WriteStdOutputToTextBox(object sender, ProgressChangedEventArgs e)
        {
            //consoleTextBox.AppendText((string)e.UserState);
        }

        private void LaunchServer(string pipeName)
        {
            try
            {
                Console.WriteLine("Creating server: " + pipeName);
                server = new NamedPipeServerStream(pipeName, PipeDirection.InOut, 4);
                Console.WriteLine("Waiting for connection");
                server.WaitForConnection();
                reader = new StreamReader(server);

                Task.Factory.StartNew(() =>
                {
                    Console.WriteLine("Begin server read loop");
                    while (true)
                    {
                        try
                        {
                            var line = reader.ReadLine();
                            messages.Enqueue(line);
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine("ServerLoop exception: {0}", ex.Message);
                        }
                    }
                });
            }
            catch (Exception ex)
            {
                Console.WriteLine("LaunchServer exception: {0}", ex.Message);
            }
        }

        public void SendMessage(string message)
        {
            if (ignoreMessages) return;
            if (writer != null)
            {
                try
                {
                    writer.Write(message);
                    writer.Flush();
                }
                catch (Exception ex)
                {
                    Console.WriteLine("SendMessage exception: {0}", ex.Message);
                }
            }
        }

        private void ProcessMessage(string message) {

          // Break apart string into separate commands
          foreach (string s in message.Split(';')) {
            var parts = s.Split(' ');
            if (parts.Length == 2) {
              // Process "stateUpdate"
              if (parts[0] == "stateUpdate") {
                ignoreMessages = true;
                foreach (string a in parts[1].Split('|')) {
                  string[] fields = a.Split('=');
                  if (fields.Length == 2) {
                    var state = fields[0];
                    var value = fields[1];
                    if (state == "time") {
                      double t = Double.Parse(value);
                      Bitmap im = new Bitmap(sliderPicBar.Width, sliderPicBar.Height);
                      sliderPicBar.Image = im;
                      using (Graphics G = Graphics.FromImage(sliderPicBar.Image)) {
                        G.Clear(Color.White);
                        double mid = t * sliderPicBar.Width;
                        int halfW = 5;
                        int xStart = (int)Math.Max(Math.Round(mid - halfW), 0);
                        int xEnd = (int)Math.Min(Math.Round(mid + halfW), sliderPicBar.Width - 1);
                        SolidBrush redBrush = new SolidBrush(Color.DarkRed);
                        Rectangle rect = new Rectangle(xStart, 0, 2 * halfW, sliderPicBar.Height);
                        G.FillRectangle(redBrush, rect);
                      }
                    } else if (state == "iSelectSeg") {
                      // Selected segment has been updated
                      selectSegIdUpDown.Value = int.Parse(value);
                    } else if (state == "showSegmentation") {
                      // Segmentation flag has been updated
                      showSegmentationCheckbox.Checked = bool.Parse(value);
                    } else if (state == "showScanVoxels") {
                      // showScanVoxels flag has been updated
                      showScanVoxelsCheckBox.Checked = bool.Parse(value);
                    } else if (state == "showScanLabeledVoxels") {
                      // showScanLabeledVoxels flag has been updated
                      showScanLabeledVoxelsCheckBox.Checked = bool.Parse(value);
                    } else if (state == "interactionSetsFile") { 
                      // Update interaction sets
                      string filename = value.Replace("/", "\\");
                      loadInteractionSets(filename);
                    } else if (state == "interactionSet") {
                      string[] vs = value.Split(':'); 
                      setInteractionSet(vs[0], vs[1]);
                    }
                  }
                }
                UpdateState();
                ignoreMessages = false;
              }
              // Process "stateInit"
              else if (parts[0] == "stateInit")
              {
                ignoreMessages = true;
                foreach (string a in parts[1].Split('|')) {
                  string[] fields = a.Split('=');
                  if (fields.Length == 2) {
                    var state = fields[0];
                    var value = fields[1];
                    if (state == "recDuration") {
                      recDurationInSec = (float)Double.Parse(value);
                      recDurationLabel.Text = "Duration:" + recDurationInSec + "s";
                      timeStartSec = 0;
                      timeEndSec = recDurationInSec;
                      timeScrollerA.Value = 0;
                      timeScrollerB.Value = timeScrollerB.Maximum;
                    } else if (state == "recId") {
                      recIdLabel.Text = value;
                    } else if (state == "recInteractionsFilename") {
                      // Update interaction annotations
                      bool loaded = loadInteractionAnnotations(value);
                      if (!loaded) {
                        // Clear annotations if there was an error loading from file
                        clearAnnotations();
                      }
                    } else if (state == "recDir") {
                      recDir = value.Replace("/", "\\");
                    } else if (state == "scanId") {
                      scanId = value;
                    } else if (state == "scanDir") {
                      scanDir = value.Replace("/", "\\");
                    }
                  }
                }
                UpdateState();
                ignoreMessages = false;
              }
              // Process "annotationUpdate"
              else if (parts[0] == "annotationUpdate")
              {
                ignoreMessages = true;
                string annotationType = null;
                string action = null;
                string id = null;
                string label = null;
                string info = "";
                string objectId = null;
                foreach (string a in parts[1].Split('|'))
                {
                  string[] fields = a.Split('=');
                  if (fields.Length == 2) {
                    var state = fields[0];
                    var value = fields[1];
                    if (state == "annoType") {
                      annotationType = value;
                    } else if (state == "action") {
                      action = value;
                    } else if (state == "idx") {
                      id = value;
                    } else if (state == "label") {
                      label = value;
                    } else if (state == "info") {
                      info = value;
                    } else if (state == "objectId") {
                      objectId = value;
                    }
                  }
                }
                if (annotationType == "segGroup") {
                  segmentAnnotationWindow.process(action, id, label, info, objectId);
                }
                ignoreMessages = false;
              }
            }
          }
        }

        private void LaunchClient(string pipeName)
        {
            try
            {
                Console.WriteLine("Creating client: " + pipeName);
                client = new NamedPipeClientStream(pipeName);
                Console.WriteLine("Connecting to client");
                client.Connect();
                writer = new StreamWriter(client);
            }
            catch (Exception ex)
            {
                Console.WriteLine("LaunchClient exception: {0}", ex.Message);
            }
        }

        private void timerProcessMessages_Tick(object sender, EventArgs e)
        {
            string message;
            if (messages.TryDequeue(out message) && message != null)
            {
                ProcessMessage(message);
            }
        }

        private void timerInitialize_Tick(object sender, EventArgs e)
        {
            timerInitialize.Enabled = false;

            LaunchClient(pipeBaseName + "ReadFromUI");
            LaunchServer(pipeBaseName + "WriteToUI");

            this.Text = "Scene Understanding UI";
        }

        private void UIWindow_Load(object sender, EventArgs e)
        {
          Screen[] screens = Screen.AllScreens;
          if (screens.Length > 1) {
            for (int i = 0; i < screens.Length; ++i) {
              if (!screens[i].Primary) {
                this.Left = screens[i].WorkingArea.Right - this.Width;
                break;
              }
            }
          } else {
            this.Left = System.Windows.Forms.Screen.PrimaryScreen.WorkingArea.Right - this.Width;
          }
          this.Top = 60;
          this.KeyPreview = true;
        }

        private void UpdateState()
        {
          if (scanDir != null) {
            segmentAnnotationWindow.setDirectory(scanDir);
          }
          if (scanId != null) {
            segmentAnnotationWindow.setFilename(scanId + ".segs");
          }
        }

        private void timerDisconnectCheck_Tick(object sender, EventArgs e)
        {
            if (server != null && reader != null && !server.IsConnected)
            {
                Application.Exit();
            }
        }

        private void UIWindow_FormClosing(object sender, FormClosingEventArgs e)
        {
            SendMessage("terminate");
        }

        private void showSkeletonCheckbox_CheckedChanged(object sender, EventArgs e)
        {
            SendMessage("D3D:showSkeletons " + showSkeletonCheckbox.Checked);
        }

        private void showInteractionFrameCheckbox_CheckedChanged(object sender, EventArgs e)
        {
          SendMessage("D3D:showInteractionFrame " + showInteractionFrameCheckbox.Checked);
        }

        //private void averageSkeletonsCheckbox_CheckedChanged(object sender, EventArgs e)
        //{
        //    SendMessage("D3D:showAverageSkeleton " + averageSkeletonsCheckbox.Checked);
        //}

        private void showGazeCheckbox_CheckedChanged(object sender, EventArgs e)
        {
            SendMessage("D3D:showGaze " + showGazeCheckbox.Checked);
        }

        private void cameraUpDown_ValueChanged(object sender, EventArgs e)
        {
            SendMessage("D3D:setCamera " + cameraUpDown.Value);
        }

        private void activationMapCheckbox_CheckedChanged(object sender, EventArgs e)
        {
            SendMessage("D3D:showActivationMap " + activationMapCheckbox.Checked);
        }

        private void paintAllJointsCheckbox_CheckedChanged(object sender, EventArgs e)
        {
            SendMessage("showAllJointsActivation " + paintAllJointsCheckbox.Checked);
        }

        private void UIWindow_KeyUp(object sender, KeyEventArgs e)
        {
            if (e.KeyCode == Keys.Escape)
            {
                SendMessage("terminate");
                Close();
            }
        }

        private void integrationWindowUpDown_ValueChanged_1(object sender, EventArgs e)
        {
            SendMessage("D3D:integrationWindow " + integrationWindowUpDown.Value);
        }

        private void jointIndexUpDown_ValueChanged(object sender, EventArgs e)
        {
            SendMessage("D3D:jointIndex " + jointIndexUpDown.Value);
        }

        private void activationRadiusUpDown_ValueChanged(object sender, EventArgs e)
        {
            SendMessage("D3D:activationRadius " + activationRadiusUpDown.Value);
        }

        private void hideUnactivatedCheckbox_CheckedChanged(object sender, EventArgs e)
        {
            SendMessage("D3D:hideUnactivated " + hideUnactivatedCheckbox.Checked);
        }

        private void showSegmentationCheckbox_CheckedChanged(object sender, EventArgs e)
        {
            SendMessage("D3D:showSegmentation " + showSegmentationCheckbox.Checked);
        }

        private void segKthreshUpDown_ValueChanged(object sender, EventArgs e)
        {
            SendMessage("D3D:segmentationKthresh " + segKthreshUpDown.Value);
        }

        private void minVertsUpDown_ValueChanged(object sender, EventArgs e)
        {
            SendMessage("D3D:segmentationMinSegVertices " + minVertsUpDown.Value);
        }

        private void timeScroller_Scroll(object sender, ScrollEventArgs e)
        {
            timeStartSec = (float)e.NewValue / timeScrollerA.Maximum * recDurationInSec;
            if ((!showColorFramesCheckbox.Checked && !depthFramesCheckbox.Checked) || e.Type == ScrollEventType.EndScroll)
            {
                SendMessage("D3D:timeScrollA " + (double)e.NewValue / timeScrollerA.Maximum);
                ignoreScrollEvent = true;
                annotationsBox.Items[annotationsBox.SelectedIndex] = Tuple.Create(timeStartSec, timeEndSec, annotationTextbox.Text);
                ignoreScrollEvent = false;
            }
        }

        private void timeScrollerB_Scroll(object sender, ScrollEventArgs e)
        {
            timeEndSec = (float)e.NewValue / timeScrollerB.Maximum * recDurationInSec;
            if ((!showColorFramesCheckbox.Checked && !depthFramesCheckbox.Checked) || e.Type == ScrollEventType.EndScroll)
            {
                SendMessage("D3D:timeScrollB " + (double)e.NewValue / timeScrollerB.Maximum);
                ignoreScrollEvent = true;
                annotationsBox.Items[annotationsBox.SelectedIndex] = Tuple.Create(timeStartSec, timeEndSec, annotationTextbox.Text);
                ignoreScrollEvent = false;
            }
        }

        private void showOBBsCheckBox_CheckedChanged(object sender, EventArgs e)
        {
            SendMessage("D3D:showSegmentOBBs " + showOBBsCheckBox.Checked);
        }

        private void segGroupOBBsCheckBox_CheckedChanged(object sender, EventArgs e)
        {
          SendMessage("D3D:showSegmentGroupOBBs " + showSegGroupOBBsCheckBox.Checked);
        }

        private void showObjOBBsCheckBox_CheckedChanged(object sender, EventArgs e)
        {
          SendMessage("D3D:showObjectOBBs " + showObjOBBsCheckBox.Checked);
        }

        private void showScanCheckBox_CheckedChanged(object sender, EventArgs e)
        {
          SendMessage("D3D:showScan " + showScanCheckBox.Checked);
        }

        private void showScanLabeledVoxels_CheckedChanged(object sender, EventArgs e)
        {
          SendMessage("D3D:setShowScanLabeledVoxels " + showScanLabeledVoxelsCheckBox.Checked);
        }

        private void showScanVoxelsCheckBox_CheckedChanged(object sender, EventArgs e)
        {
          SendMessage("D3D:setShowScanVoxels " + showScanVoxelsCheckBox.Checked);
        }

        private void showBodyPointCheckBox_CheckedChanged(object sender, EventArgs e)
        {
          SendMessage("D3D:showBodyPoints " + showBodyPointCheckBox.Checked);
        }

        private void segSaveButton_Click(object sender, EventArgs e)
        {
            string path;
            SaveFileDialog file = new SaveFileDialog();
            file.DefaultExt = "csv";
            file.InitialDirectory = Environment.CurrentDirectory;

            if (file.ShowDialog() == DialogResult.OK)
            {
                path = file.FileName;
                SendMessage("D3D:saveSeg " + path);
            }
        }


        private void segLoadButton_Click(object sender, EventArgs e)
        {
            string path;
            OpenFileDialog file = new OpenFileDialog();
            file.Filter = "CSV files (*.csv)|*.csv|All files (*.*)|*.*";
            file.InitialDirectory = Environment.CurrentDirectory;

            if (file.ShowDialog() == DialogResult.OK)
            {
                path = file.FileName;
                SendMessage("D3D:loadSeg " + path);
            }
        }


        private void segColorWeightUpDown_ValueChanged(object sender, EventArgs e)
        {
            SendMessage("D3D:segColorWeight " + segColorWeightUpDown.Value);
        }

        private void selectSegIdUpDown_ValueChanged(object sender, EventArgs e)
        {
            SendMessage("D3D:selectSegId " + selectSegIdUpDown.Value);
        }

        private void minSegDiagUpDown_ValueChanged(object sender, EventArgs e)
        {
            SendMessage("D3D:minSegDiag " + minSegDiagUpDown.Value);
        }

        private void showInteractionButton_CheckedChanged(object sender, EventArgs e)
        {
            SendMessage("D3D:showInteractionOBBs " + showInteractionButton.Checked);
        }

        private void segKnnButton_Click(object sender, EventArgs e)
        {
            SendMessage("D3D:segKnn");
        }

        private void jointSegKnnButton_Click(object sender, EventArgs e)
        {
            SendMessage("D3D:jointSegKnn");
        }

        private void showColorFramesCheckbox_CheckedChanged(object sender, EventArgs e)
        {
            SendMessage("showColorFrames " + showColorFramesCheckbox.Checked);
        }

        private void depthFramesCheckbox_CheckedChanged(object sender, EventArgs e)
        {
            SendMessage("showDepthFrames " + depthFramesCheckbox.Checked);
        }

        // Interaction annotation changes

        // Pushes annotation onto the stack
        private void annotationPushButton_Click(object sender, EventArgs e)
        {
            if (annotationTextbox.Text.Length > 0)
            {
                float ratioA = (float)timeScrollerA.Value / timeScrollerA.Maximum;
                float ratioB = (float)timeScrollerB.Value / timeScrollerB.Maximum;
                float timeA = recDurationInSec * ratioA;
                float timeB = recDurationInSec * ratioB;
                var ann = Tuple.Create(timeA, timeB, annotationTextbox.Text);
                annotationsBox.Items[annotationsBox.SelectedIndex] = ann;
                annotationsBox.SelectedIndex = annotationsBox.Items.Count - 1;
                if (annotationsBox.SelectedItem != newAnnGuard)
                {
                    annotationsBox.Items.Add(newAnnGuard);
                    annotationsBox.SelectedIndex = annotationsBox.Items.Count - 1;
                }
                annotationTextbox.Text = "";
            }
        }

        // Dumps annotation to a file
        private void annotationDumpButton_Click(object sender, EventArgs e)
        {
            string path;
            SaveFileDialog file = new SaveFileDialog();
            file.DefaultExt = "interactions.txt";
            file.Filter = "Interaction files (*.interactions.txt)|*.interactions.txt|Text files (*.txt)|*.txt|All files (*.*)|*.*";
            file.InitialDirectory = recDir;
            file.FileName = recIdLabel.Text;
            if (file.ShowDialog() == DialogResult.OK)
            {
                path = file.FileName;
                StreamWriter writer = new System.IO.StreamWriter(path);
                foreach (Tuple<float, float, string> ann in annotationsBox.Items)
                {
                    if (ann == newAnnGuard) { continue; }
                    writer.WriteLine(ann.Item1 + "-" + ann.Item2 + "," + ann.Item3);
                }
                writer.Close();
            }
        }

        // Plays interaction
        private void playRangeButton_CheckedChanged(object sender, EventArgs e)
        {
            // automatically switch into replay mode
            setReplayMode();
            double ratioA = (double)timeScrollerA.Value / timeScrollerA.Maximum;
            double ratioB = (double)timeScrollerB.Value / timeScrollerB.Maximum;
            SendMessage("D3D:playRange " + ratioA + "," + ratioB);
        }

        private void annotationTextbox_TextChanged(object sender, EventArgs e)
        {
            //annotationsBox.Items[annotationsBox.SelectedIndex] = Tuple.Create(timeStartSec, timeEndSec, annotationTextbox.Text);
        }

        private void annotationsBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (ignoreScrollEvent) { return; }
            if (annotationsBox.SelectedItem == null) { return; }
            Tuple<float, float, string> ann = (Tuple<float, float, string>)annotationsBox.SelectedItem;
            if (ann.Item1 > 0) timeScrollerA.Value = (int)(ann.Item1 / recDurationInSec * timeScrollerA.Maximum);
            if (ann.Item2 > 0) timeScrollerB.Value = (int)(ann.Item2 / recDurationInSec * timeScrollerB.Maximum);
            if (ann.Item3 != "New") annotationTextbox.Text = ann.Item3;
            if (timeScrollerA.Value >= 0)
            {
              // automatically switch into replay mode
              setReplayMode();
              SendMessage("D3D:timeScrollA " + (double)timeScrollerA.Value / timeScrollerA.Maximum);
            }
        }

        // Delete annotation
        private void annDeleteButton_Click(object sender, EventArgs e)
        {
            if (annotationsBox.SelectedItem == newAnnGuard) { return; }
            annotationsBox.Items.RemoveAt(annotationsBox.SelectedIndex);
            annotationsBox.SelectedIndex = annotationsBox.Items.Count - 1;
        }

        private void createDBbutton_Click(object sender, EventArgs e)
        {
            SendMessage("D3D:createDatabase");
        }

        private void nextPoseButton_Click(object sender, EventArgs e)
        {
            // automatically switch into pose mode
            setPoseMode();
            SendMessage("D3D:nextPose");
        }

        private void nextISetButton_Click(object sender, EventArgs e)
        {
          // automatically switch into pose mode
          setPoseMode();
          SendMessage("D3D:nextInteractionSet");
        }


        private void showInteractionHeatmap_CheckedChanged(object sender, EventArgs e)
        {
            SendMessage("D3D:showInteractionHeatmap " + showInteractionHeatmap.Checked);
        }

        private void showSegVerbProbButton_CheckedChanged(object sender, EventArgs e)
        {
            SendMessage("D3D:showSegmentVerbProbability " + showSegVerbProbButton.Checked);
        }

        private void recomputeHeatmapButton_Click(object sender, EventArgs e)
        {
          // Make sure that the heatmap is enabled
          if (!showInteractionHeatmap.Checked) {
            showInteractionHeatmap.Checked = true;
            SendMessage("D3D:showInteractionHeatmap " + showInteractionHeatmap.Checked);
          }
          SendMessage("D3D:updateHeatmap");
        }

        private void interactionHeatmapUseMax_CheckedChanged(object sender, EventArgs e)
        {
            SendMessage("heatmapNormalize " + interactionHeatmapUseMax.Checked);
        }

        private void jointSegKnnUseCurr_CheckedChanged(object sender, EventArgs e)
        {
            SendMessage("jointSegKnnUseCurr " + jointSegKnnUseCurr.Checked);
        }

        private void segPerJointClassifyButton_Click(object sender, EventArgs e)
        {
            SendMessage("D3D:segPerJointClassify");
        }

        private void modeRadio_CheckedChanged(object sender, EventArgs e)
        {
          if (poseModeRadio.Checked) {
            SendMessage("D3D:setMode pose");
            SendMessage("D3D:setPoseInteractionSet "
              + interactionTypeDropdown.SelectedItem.ToString() + " "
              + interactionDropdown.SelectedItem.ToString());
          } else if (replayModeRadio.Checked) {
            SendMessage("D3D:setMode replay");
          }
        }

        private void checkBoxShowClusterAssignment_CheckedChanged(object sender, EventArgs e)
        {
            SendMessage("D3D:showClusterAssignment " + checkBoxShowClusterAssignment.Checked);
        }

        private void interactionHeatmapShowAngle_CheckedChanged(object sender, EventArgs e)
        {
            SendMessage("heatmapShowAngle " + interactionHeatmapShowAngle.Checked);
        }

        private void nextRecordingButton_Click(object sender, EventArgs e)
        {
            SendMessage("D3D:nextRecording");
        }

        private void classifierTypeDropdown_SelectedIndexChanged(object sender, EventArgs e)
        {
          SendMessage("currentClassifierType " + classifierTypeDropdown.SelectedItem);
        }

        private void constrainZupCheckbox_CheckedChanged(object sender, EventArgs e)
        {
            SendMessage("D3D:segConstrainZup " + constrainZupCheckbox.Checked);
        }

        private void useSingleSkelForHeatmapCheckbox_CheckedChanged(object sender, EventArgs e)
        {
            SendMessage("useSingleSkelForHeatmap " + useSingleSkelForHeatmapCheckbox.Checked);
        }

        // Loads annotation
        private void loadAnnotationButton_Click(object sender, EventArgs e)
        {
            OpenFileDialog file = new OpenFileDialog();
            file.Filter = "Interaction files (*.interactions.txt)|*.interactions.txt|Text files (*.txt)|*.txt|All files (*.*)|*.*";
            file.InitialDirectory = recDir;
            if (file.ShowDialog() == DialogResult.OK)
            {
                // Read annotations
                loadInteractionAnnotations(file.FileName);
            }
        }

        private void trainButton_Click(object sender, EventArgs e)
        {
            SendMessage("D3D:trainSelected");
        }

        private void trainAllButton_Click(object sender, EventArgs e)
        {
            SendMessage("D3D:trainAll");
        }

        private void testButton_Click(object sender, EventArgs e)
        {
            SendMessage("D3D:testSelected");
        }

        private void testInteractionButton_Click(object sender, EventArgs e)
        {
            SendMessage("D3D:testInteraction");
        }

        private void cvButton_Click(object sender, EventArgs e)
        {
            SendMessage("D3D:crossValidate");
        }

        private void buttonDumpMesh_Click(object sender, EventArgs e)
        {
            SendMessage("D3D:dumpMesh");
        }

        private void buttonLoad_Click(object sender, EventArgs e)
        {
            SendMessage("D3D:loadScan " + textBoxLoad.Text);
        }

        private void dumpImagesCheckbox_CheckedChanged(object sender, EventArgs e)
        {
            SendMessage("D3D:dumpTestImages " + dumpImagesCheckbox.Checked);
        }

        private void helpButton_Click(object sender, EventArgs e)
        {
          System.Diagnostics.Process.Start("https://github.com/msavva/scenegrok/wiki/SceneGrok-UI");
        }

        private void showKNNIGSimCheckbox_CheckedChanged(object sender, EventArgs e)
        {
            SendMessage("showKnnIGSim " + showKNNIGSimCheckbox.Checked);
        }

        private void showPIGSimCheckbox_CheckedChanged(object sender, EventArgs e)
        {
            SendMessage("showPIGSim " + showPIGSimCheckbox.Checked);
        }

        private void showModelsCheckbox_CheckedChanged(object sender, EventArgs e)
        {
          SendMessage("D3D:showModels " + showModelsCheckbox.Checked);
        }

        private void loadModelButton_Click(object sender, EventArgs e)
        {
          SendMessage("D3D:loadModel " + modelLoadTypeDropdown.SelectedItem + " " + modelTextBoxLoad.Text);
        }

        private void showModelInteractionMapCheckbox_CheckedChanged(object sender, EventArgs e)
        {
          SendMessage("D3D:showModelInteractionMaps " + showModelInteractionMapCheckbox.Checked);
        }

      private void showModelSegmentationCheckbox_CheckedChanged(object sender, EventArgs e)
        {
          SendMessage("D3D:showModelSegmentation " + showModelSegmentationCheckbox.Checked);
        }

        private void loadModelSegFromFileCheckBox_CheckedChanged(object sender, EventArgs e)
        {
          SendMessage("D3D:loadModelSegmentation " + loadModelSegFromFileCheckBox.Checked);
        }

        private void showModelVoxelsCheckBox_CheckedChanged(object sender, EventArgs e)
        {
          SendMessage("D3D:showModelVoxels " + showModelVoxelsCheckBox.Checked);
        }

        private void interactionTypeDropdown_SelectedIndexChanged(object sender, EventArgs e)
        {
          populateInteractionSets(interactionTypeDropdown.SelectedItem.ToString(), showAllInteractionSetsCheckBox.Checked);
        }

        private void interactionDropdown_SelectedIndexChanged(object sender, EventArgs e)
        {
          // automatically switch into pose mode
          if (!initializingInteractionSets) setPoseMode();
          SendMessage("D3D:setPoseInteractionSet " + interactionTypeDropdown.SelectedItem.ToString() + " " 
            + interactionDropdown.SelectedItem.ToString());
        }

        // Helper functions

        private void createAnnotationWindows()
        {
            segmentAnnotationWindow = new AnnotatorWindow(this, "segGroup");
            segmentAnnotationWindow.Text = "Segmentation Annotations";
        }

        private void segAnnotateButton_Click(object sender, EventArgs e)
        {
            segmentAnnotationWindow.Show();
        }

        private bool loadInteractionAnnotations(string path)
        {
            if (path != "" && System.IO.File.Exists(path))
            {
                // Reads file and load annotations
                StreamReader reader = new System.IO.StreamReader(path);
                string line;
                annotationsBox.Items.Clear();
                while ((line = reader.ReadLine()) != null)
                {
                    string[] tokens = line.Split(',');
                    if (tokens.Length < 2) { continue; }
                    string timeRange = tokens[0];
                    string annotationText = tokens[1];
                    string[] timeStartEnd = timeRange.Split('-');
                    if (timeStartEnd.Length < 2) { continue; }
                    float timeStart = float.Parse(timeStartEnd[0]);
                    float timeEnd = float.Parse(timeStartEnd[1]);
                    var ann = Tuple.Create(timeStart, timeEnd, annotationText);
                    annotationsBox.Items.Add(ann);
                }
                reader.Close();
                annotationsBox.Items.Add(newAnnGuard);
                annotationsBox.SelectedIndex = 0;
                return true;
            }
            else
            {
                // Loading annotation from invalid path
                if (path == "")
                {
                    Console.WriteLine("Cannot load interaction annotations from empty path");
                }
                else
                {
                    Console.WriteLine("Error loading interaction annotations from invalid path: " + path);
                }
                return false;
            }
        }

        private void clearAnnotations()
        {
            annotationsBox.Items.Clear();
            annotationsBox.Items.Add(newAnnGuard);
            annotationsBox.SelectedIndex = 0;
        }


        private void showAllInteractionSetsCheckBox_CheckedChanged(object sender, EventArgs e)
        {
          populateInteractionSets(interactionTypeDropdown.SelectedItem.ToString(),
            showAllInteractionSetsCheckBox.Checked);
        }

        private bool loadInteractionSets(string path)
        {
          if (path != "" && System.IO.File.Exists(path))
          {
            // Reads file and load annotations
            interactionSets = new Dictionary<string, List<string>>();
            interactionSetsForRecording = new Dictionary<string, Dictionary<string, List<string>>>();
            StreamReader reader = new System.IO.StreamReader(path);
            string line;
            string header = reader.ReadLine();
            while ((line = reader.ReadLine()) != null)
            {
              string[] tokens = line.Split(',');
              if (tokens.Length < 5) { continue; }
              string type = tokens[0];
              string id = tokens[1];
              string verbs = tokens[2];
              string nouns = tokens[3];
              //string count = tokens[4];
              string recording = tokens[4];
              // Add to overall interaction sets
              if (!interactionSets.ContainsKey(type)) {
                interactionSets.Add(type, new List<string>());
              }
              List<string> interactions = interactionSets[type];
              if (!interactions.Contains(id)) {
                interactions.Add(id);
              }
              // Add to interaction set for recording
              if (!interactionSetsForRecording.ContainsKey(recording)) {
                interactionSetsForRecording.Add(recording, new Dictionary<string, List<String>>());
              }
              Dictionary<string, List<String>> iSets = interactionSetsForRecording[recording];
              if (!iSets.ContainsKey(type)) {
                iSets.Add(type, new List<string>());
              }
              interactions = iSets[type];
              if (!interactions.Contains(id)) {
                interactions.Add(id);
              }
            }
            reader.Close();
            populateInteractionSets(interactionTypeDropdown.SelectedItem.ToString(), 
              showAllInteractionSetsCheckBox.Checked);
            return true;
          }
          else
          {
            // Loading annotation from invalid path
            if (path == "")
            {
              Console.WriteLine("Cannot load interaction sets from empty path");
            }
            else
            {
              Console.WriteLine("Error loading interaction sets from invalid path: " + path);
            }
            return false;
          }
        }

        private void populateInteractionSets(string type, bool useAllRecordings)
        {
          initializingInteractionSets = true;
          List<string> interactions;
          Dictionary<string, List<string>> interSets = interactionSets;
          if (!useAllRecordings && interactionSetsForRecording != null)
          {
            string currentRecordingId = recIdLabel.Text;
            if (interactionSetsForRecording.ContainsKey(currentRecordingId)) {
              interactionSetsForRecording.TryGetValue(currentRecordingId, out interSets);
            }
          }
          if (interSets != null && interSets.TryGetValue(type, out interactions))
          {
            interactionDropdown.Items.Clear();
            interactionDropdown.Items.AddRange(interactions.ToArray());
            interactionDropdown.SelectedIndex = 0;
          }
          initializingInteractionSets = false;
        }

        private void setInteractionSet(string type, string id)
        {
          int iType = interactionTypeDropdown.Items.IndexOf(type);
          if (iType >= 0) {
            if (iType != interactionTypeDropdown.SelectedIndex) {
              interactionTypeDropdown.SelectedIndex = iType;
              populateInteractionSets(type, showAllInteractionSetsCheckBox.Checked);
            }
            int iInteraction = interactionDropdown.Items.IndexOf(id);
            if (iInteraction >= 0) {
              interactionDropdown.SelectedIndex = iInteraction;
            }
          }
        }

        private void setPoseMode()
        {
          if (!poseModeRadio.Checked) {
            replayModeRadio.Checked = false;
            poseModeRadio.Checked = true;
            SendMessage("D3D:setMode pose");
          }
        }

        private void setReplayMode()
        {
          if (!replayModeRadio.Checked)
          {
            poseModeRadio.Checked = false;
            replayModeRadio.Checked = true;
            SendMessage("D3D:setMode replay");
          }
        }
    }
}
