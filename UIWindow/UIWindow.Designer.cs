namespace UIWindow
{
    partial class UIWindow
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
      this.components = new System.ComponentModel.Container();
      this.timerProcessMessages = new System.Windows.Forms.Timer(this.components);
      this.groupBox1 = new System.Windows.Forms.GroupBox();
      this.showObjOBBsCheckBox = new System.Windows.Forms.CheckBox();
      this.showSegGroupOBBsCheckBox = new System.Windows.Forms.CheckBox();
      this.segLoadButton = new System.Windows.Forms.Button();
      this.segAnnotateButton = new System.Windows.Forms.Button();
      this.constrainZupCheckbox = new System.Windows.Forms.CheckBox();
      this.jointSegKnnUseCurr = new System.Windows.Forms.CheckBox();
      this.segKnnButton = new System.Windows.Forms.Button();
      this.jointSegKnnButton = new System.Windows.Forms.Button();
      this.minSegDiagUpDown = new System.Windows.Forms.NumericUpDown();
      this.label8 = new System.Windows.Forms.Label();
      this.selectSegIdUpDown = new System.Windows.Forms.NumericUpDown();
      this.label7 = new System.Windows.Forms.Label();
      this.segColorWeightUpDown = new System.Windows.Forms.NumericUpDown();
      this.label6 = new System.Windows.Forms.Label();
      this.segSaveButton = new System.Windows.Forms.Button();
      this.showOBBsCheckBox = new System.Windows.Forms.CheckBox();
      this.minVertsUpDown = new System.Windows.Forms.NumericUpDown();
      this.label5 = new System.Windows.Forms.Label();
      this.segKthreshUpDown = new System.Windows.Forms.NumericUpDown();
      this.kThreshLabel = new System.Windows.Forms.Label();
      this.showSegmentationCheckbox = new System.Windows.Forms.CheckBox();
      this.timerInitialize = new System.Windows.Forms.Timer(this.components);
      this.timerDisconnectCheck = new System.Windows.Forms.Timer(this.components);
      this.timeScrollerA = new System.Windows.Forms.HScrollBar();
      this.label2 = new System.Windows.Forms.Label();
      this.CameraLabel = new System.Windows.Forms.Label();
      this.cameraUpDown = new System.Windows.Forms.NumericUpDown();
      this.integrationWindowUpDown = new System.Windows.Forms.NumericUpDown();
      this.jointIndexUpDown = new System.Windows.Forms.NumericUpDown();
      this.label4 = new System.Windows.Forms.Label();
      this.actRlabel = new System.Windows.Forms.Label();
      this.activationRadiusUpDown = new System.Windows.Forms.NumericUpDown();
      this.activationMapCheckbox = new System.Windows.Forms.CheckBox();
      this.showSkeletonCheckbox = new System.Windows.Forms.CheckBox();
      this.hideUnactivatedCheckbox = new System.Windows.Forms.CheckBox();
      this.showGazeCheckbox = new System.Windows.Forms.CheckBox();
      this.showScanCheckBox = new System.Windows.Forms.CheckBox();
      this.showBodyPointCheckBox = new System.Windows.Forms.CheckBox();
      this.showInteractionButton = new System.Windows.Forms.CheckBox();
      this.timeScrollerB = new System.Windows.Forms.HScrollBar();
      this.sliderPicBar = new System.Windows.Forms.PictureBox();
      this.showColorFramesCheckbox = new System.Windows.Forms.CheckBox();
      this.depthFramesCheckbox = new System.Windows.Forms.CheckBox();
      this.groupBox2 = new System.Windows.Forms.GroupBox();
      this.paintAllJointsCheckbox = new System.Windows.Forms.CheckBox();
      this.showPIGSimCheckbox = new System.Windows.Forms.CheckBox();
      this.showKNNIGSimCheckbox = new System.Windows.Forms.CheckBox();
      this.groupBox3 = new System.Windows.Forms.GroupBox();
      this.annotationBox = new System.Windows.Forms.GroupBox();
      this.loadAnnotationButton = new System.Windows.Forms.Button();
      this.annDeleteButton = new System.Windows.Forms.Button();
      this.playRangeButton = new System.Windows.Forms.CheckBox();
      this.annotationTextbox = new System.Windows.Forms.TextBox();
      this.annotationDumpButton = new System.Windows.Forms.Button();
      this.annotationPushButton = new System.Windows.Forms.Button();
      this.annotationsBox = new System.Windows.Forms.ListBox();
      this.groupBox4 = new System.Windows.Forms.GroupBox();
      this.nextISetButton = new System.Windows.Forms.Button();
      this.testInteractionButton = new System.Windows.Forms.Button();
      this.classifierTypeDropdown = new System.Windows.Forms.ComboBox();
      this.dumpImagesCheckbox = new System.Windows.Forms.CheckBox();
      this.testButton = new System.Windows.Forms.Button();
      this.cvButton = new System.Windows.Forms.Button();
      this.trainAllButton = new System.Windows.Forms.Button();
      this.trainButton = new System.Windows.Forms.Button();
      this.segPerJointClassifyButton = new System.Windows.Forms.Button();
      this.recomputeHeatmapButton = new System.Windows.Forms.Button();
      this.useSingleSkelForHeatmapCheckbox = new System.Windows.Forms.CheckBox();
      this.interactionHeatmapShowAngle = new System.Windows.Forms.CheckBox();
      this.interactionHeatmapUseMax = new System.Windows.Forms.CheckBox();
      this.showSegVerbProbButton = new System.Windows.Forms.CheckBox();
      this.checkBoxShowClusterAssignment = new System.Windows.Forms.CheckBox();
      this.showInteractionHeatmap = new System.Windows.Forms.CheckBox();
      this.nextPoseButton = new System.Windows.Forms.Button();
      this.createDBbutton = new System.Windows.Forms.Button();
      this.nextRecordingButton = new System.Windows.Forms.Button();
      this.recIdLabel = new System.Windows.Forms.Label();
      this.buttonHeatMesh = new System.Windows.Forms.Button();
      this.textBoxLoad = new System.Windows.Forms.TextBox();
      this.buttonLoad = new System.Windows.Forms.Button();
      this.helpButton = new System.Windows.Forms.Button();
      this.showInteractionFrameCheckbox = new System.Windows.Forms.CheckBox();
      this.ModelsGroup = new System.Windows.Forms.GroupBox();
      this.showModelInteractionMapCheckbox = new System.Windows.Forms.CheckBox();
      this.showModelsCheckbox = new System.Windows.Forms.CheckBox();
      this.modelLoadTypeDropdown = new System.Windows.Forms.ComboBox();
      this.showModelVoxelsCheckBox = new System.Windows.Forms.CheckBox();
      this.loadModelSegFromFileCheckBox = new System.Windows.Forms.CheckBox();
      this.showModelSegmentationCheckbox = new System.Windows.Forms.CheckBox();
      this.modelTextBoxLoad = new System.Windows.Forms.TextBox();
      this.loadModelButton = new System.Windows.Forms.Button();
      this.recDurationLabel = new System.Windows.Forms.Label();
      this.visualizationGroupBox = new System.Windows.Forms.GroupBox();
      this.showScanLabeledVoxelsCheckBox = new System.Windows.Forms.CheckBox();
      this.showScanVoxelsCheckBox = new System.Windows.Forms.CheckBox();
      this.modeGroupbox = new System.Windows.Forms.GroupBox();
      this.showAllInteractionSetsCheckBox = new System.Windows.Forms.CheckBox();
      this.poseModeRadio = new System.Windows.Forms.RadioButton();
      this.replayModeRadio = new System.Windows.Forms.RadioButton();
      this.interactionDropdown = new System.Windows.Forms.ComboBox();
      this.interactionTypeDropdown = new System.Windows.Forms.ComboBox();
      this.groupBox1.SuspendLayout();
      ((System.ComponentModel.ISupportInitialize)(this.minSegDiagUpDown)).BeginInit();
      ((System.ComponentModel.ISupportInitialize)(this.selectSegIdUpDown)).BeginInit();
      ((System.ComponentModel.ISupportInitialize)(this.segColorWeightUpDown)).BeginInit();
      ((System.ComponentModel.ISupportInitialize)(this.minVertsUpDown)).BeginInit();
      ((System.ComponentModel.ISupportInitialize)(this.segKthreshUpDown)).BeginInit();
      ((System.ComponentModel.ISupportInitialize)(this.cameraUpDown)).BeginInit();
      ((System.ComponentModel.ISupportInitialize)(this.integrationWindowUpDown)).BeginInit();
      ((System.ComponentModel.ISupportInitialize)(this.jointIndexUpDown)).BeginInit();
      ((System.ComponentModel.ISupportInitialize)(this.activationRadiusUpDown)).BeginInit();
      ((System.ComponentModel.ISupportInitialize)(this.sliderPicBar)).BeginInit();
      this.groupBox2.SuspendLayout();
      this.groupBox3.SuspendLayout();
      this.annotationBox.SuspendLayout();
      this.groupBox4.SuspendLayout();
      this.ModelsGroup.SuspendLayout();
      this.visualizationGroupBox.SuspendLayout();
      this.modeGroupbox.SuspendLayout();
      this.SuspendLayout();
      // 
      // timerProcessMessages
      // 
      this.timerProcessMessages.Enabled = true;
      this.timerProcessMessages.Interval = 1;
      this.timerProcessMessages.Tick += new System.EventHandler(this.timerProcessMessages_Tick);
      // 
      // groupBox1
      // 
      this.groupBox1.Controls.Add(this.showObjOBBsCheckBox);
      this.groupBox1.Controls.Add(this.showSegGroupOBBsCheckBox);
      this.groupBox1.Controls.Add(this.segLoadButton);
      this.groupBox1.Controls.Add(this.segAnnotateButton);
      this.groupBox1.Controls.Add(this.constrainZupCheckbox);
      this.groupBox1.Controls.Add(this.jointSegKnnUseCurr);
      this.groupBox1.Controls.Add(this.segKnnButton);
      this.groupBox1.Controls.Add(this.jointSegKnnButton);
      this.groupBox1.Controls.Add(this.minSegDiagUpDown);
      this.groupBox1.Controls.Add(this.label8);
      this.groupBox1.Controls.Add(this.selectSegIdUpDown);
      this.groupBox1.Controls.Add(this.label7);
      this.groupBox1.Controls.Add(this.segColorWeightUpDown);
      this.groupBox1.Controls.Add(this.label6);
      this.groupBox1.Controls.Add(this.segSaveButton);
      this.groupBox1.Controls.Add(this.showOBBsCheckBox);
      this.groupBox1.Controls.Add(this.minVertsUpDown);
      this.groupBox1.Controls.Add(this.label5);
      this.groupBox1.Controls.Add(this.segKthreshUpDown);
      this.groupBox1.Controls.Add(this.kThreshLabel);
      this.groupBox1.Controls.Add(this.showSegmentationCheckbox);
      this.groupBox1.FlatStyle = System.Windows.Forms.FlatStyle.Popup;
      this.groupBox1.Location = new System.Drawing.Point(227, 103);
      this.groupBox1.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
      this.groupBox1.Name = "groupBox1";
      this.groupBox1.Padding = new System.Windows.Forms.Padding(3, 4, 3, 4);
      this.groupBox1.Size = new System.Drawing.Size(303, 190);
      this.groupBox1.TabIndex = 5;
      this.groupBox1.TabStop = false;
      this.groupBox1.Text = "Segmentation";
      // 
      // showObjOBBsCheckBox
      // 
      this.showObjOBBsCheckBox.AutoSize = true;
      this.showObjOBBsCheckBox.Location = new System.Drawing.Point(136, 139);
      this.showObjOBBsCheckBox.Name = "showObjOBBsCheckBox";
      this.showObjOBBsCheckBox.Size = new System.Drawing.Size(85, 23);
      this.showObjOBBsCheckBox.TabIndex = 54;
      this.showObjOBBsCheckBox.Text = "objOBBs";
      this.showObjOBBsCheckBox.UseVisualStyleBackColor = true;
      this.showObjOBBsCheckBox.CheckedChanged += new System.EventHandler(this.showObjOBBsCheckBox_CheckedChanged);
      // 
      // showSegGroupOBBsCheckBox
      // 
      this.showSegGroupOBBsCheckBox.AutoSize = true;
      this.showSegGroupOBBsCheckBox.Location = new System.Drawing.Point(10, 138);
      this.showSegGroupOBBsCheckBox.Name = "showSegGroupOBBsCheckBox";
      this.showSegGroupOBBsCheckBox.Size = new System.Drawing.Size(128, 23);
      this.showSegGroupOBBsCheckBox.TabIndex = 53;
      this.showSegGroupOBBsCheckBox.Text = "segGroupOBBs";
      this.showSegGroupOBBsCheckBox.UseVisualStyleBackColor = true;
      this.showSegGroupOBBsCheckBox.CheckedChanged += new System.EventHandler(this.segGroupOBBsCheckBox_CheckedChanged);
      // 
      // segLoadButton
      // 
      this.segLoadButton.Location = new System.Drawing.Point(149, 163);
      this.segLoadButton.Name = "segLoadButton";
      this.segLoadButton.Size = new System.Drawing.Size(75, 26);
      this.segLoadButton.TabIndex = 48;
      this.segLoadButton.Text = "Load";
      this.segLoadButton.UseVisualStyleBackColor = true;
      this.segLoadButton.Click += new System.EventHandler(this.segLoadButton_Click);
      // 
      // segAnnotateButton
      // 
      this.segAnnotateButton.Location = new System.Drawing.Point(9, 162);
      this.segAnnotateButton.Name = "segAnnotateButton";
      this.segAnnotateButton.Size = new System.Drawing.Size(95, 26);
      this.segAnnotateButton.TabIndex = 47;
      this.segAnnotateButton.Text = "Annotate";
      this.segAnnotateButton.UseVisualStyleBackColor = true;
      this.segAnnotateButton.Click += new System.EventHandler(this.segAnnotateButton_Click);
      // 
      // constrainZupCheckbox
      // 
      this.constrainZupCheckbox.AutoSize = true;
      this.constrainZupCheckbox.Checked = true;
      this.constrainZupCheckbox.CheckState = System.Windows.Forms.CheckState.Checked;
      this.constrainZupCheckbox.Location = new System.Drawing.Point(151, 90);
      this.constrainZupCheckbox.Name = "constrainZupCheckbox";
      this.constrainZupCheckbox.Size = new System.Drawing.Size(65, 23);
      this.constrainZupCheckbox.TabIndex = 46;
      this.constrainZupCheckbox.Text = "ZisUp";
      this.constrainZupCheckbox.UseVisualStyleBackColor = true;
      this.constrainZupCheckbox.CheckedChanged += new System.EventHandler(this.constrainZupCheckbox_CheckedChanged);
      // 
      // jointSegKnnUseCurr
      // 
      this.jointSegKnnUseCurr.AutoSize = true;
      this.jointSegKnnUseCurr.Location = new System.Drawing.Point(159, 120);
      this.jointSegKnnUseCurr.Name = "jointSegKnnUseCurr";
      this.jointSegKnnUseCurr.Size = new System.Drawing.Size(57, 23);
      this.jointSegKnnUseCurr.TabIndex = 45;
      this.jointSegKnnUseCurr.Text = "Curr";
      this.jointSegKnnUseCurr.UseVisualStyleBackColor = true;
      this.jointSegKnnUseCurr.CheckedChanged += new System.EventHandler(this.jointSegKnnUseCurr_CheckedChanged);
      // 
      // segKnnButton
      // 
      this.segKnnButton.Location = new System.Drawing.Point(222, 90);
      this.segKnnButton.Name = "segKnnButton";
      this.segKnnButton.Size = new System.Drawing.Size(75, 26);
      this.segKnnButton.TabIndex = 39;
      this.segKnnButton.Text = "kNN";
      this.segKnnButton.UseVisualStyleBackColor = true;
      this.segKnnButton.Click += new System.EventHandler(this.segKnnButton_Click);
      // 
      // jointSegKnnButton
      // 
      this.jointSegKnnButton.Location = new System.Drawing.Point(222, 117);
      this.jointSegKnnButton.Name = "jointSegKnnButton";
      this.jointSegKnnButton.Size = new System.Drawing.Size(75, 26);
      this.jointSegKnnButton.TabIndex = 38;
      this.jointSegKnnButton.Text = "JointSeg";
      this.jointSegKnnButton.UseVisualStyleBackColor = true;
      this.jointSegKnnButton.Click += new System.EventHandler(this.jointSegKnnButton_Click);
      // 
      // minSegDiagUpDown
      // 
      this.minSegDiagUpDown.DecimalPlaces = 2;
      this.minSegDiagUpDown.Increment = new decimal(new int[] {
            1,
            0,
            0,
            131072});
      this.minSegDiagUpDown.Location = new System.Drawing.Point(236, 57);
      this.minSegDiagUpDown.Maximum = new decimal(new int[] {
            10,
            0,
            0,
            0});
      this.minSegDiagUpDown.Name = "minSegDiagUpDown";
      this.minSegDiagUpDown.Size = new System.Drawing.Size(61, 27);
      this.minSegDiagUpDown.TabIndex = 36;
      this.minSegDiagUpDown.Value = new decimal(new int[] {
            15,
            0,
            0,
            131072});
      this.minSegDiagUpDown.ValueChanged += new System.EventHandler(this.minSegDiagUpDown_ValueChanged);
      // 
      // label8
      // 
      this.label8.AutoSize = true;
      this.label8.Location = new System.Drawing.Point(161, 59);
      this.label8.Name = "label8";
      this.label8.Size = new System.Drawing.Size(69, 19);
      this.label8.TabIndex = 35;
      this.label8.Text = "minDiag:";
      // 
      // selectSegIdUpDown
      // 
      this.selectSegIdUpDown.Location = new System.Drawing.Point(236, 26);
      this.selectSegIdUpDown.Maximum = new decimal(new int[] {
            10000,
            0,
            0,
            0});
      this.selectSegIdUpDown.Minimum = new decimal(new int[] {
            100000,
            0,
            0,
            -2147483648});
      this.selectSegIdUpDown.Name = "selectSegIdUpDown";
      this.selectSegIdUpDown.Size = new System.Drawing.Size(61, 27);
      this.selectSegIdUpDown.TabIndex = 34;
      this.selectSegIdUpDown.ValueChanged += new System.EventHandler(this.selectSegIdUpDown_ValueChanged);
      // 
      // label7
      // 
      this.label7.AutoSize = true;
      this.label7.Location = new System.Drawing.Point(152, 28);
      this.label7.Name = "label7";
      this.label7.Size = new System.Drawing.Size(82, 19);
      this.label7.TabIndex = 33;
      this.label7.Text = "iSelectSeg:";
      // 
      // segColorWeightUpDown
      // 
      this.segColorWeightUpDown.DecimalPlaces = 2;
      this.segColorWeightUpDown.Increment = new decimal(new int[] {
            1,
            0,
            0,
            131072});
      this.segColorWeightUpDown.Location = new System.Drawing.Point(79, 107);
      this.segColorWeightUpDown.Maximum = new decimal(new int[] {
            1,
            0,
            0,
            0});
      this.segColorWeightUpDown.Name = "segColorWeightUpDown";
      this.segColorWeightUpDown.Size = new System.Drawing.Size(61, 27);
      this.segColorWeightUpDown.TabIndex = 32;
      this.segColorWeightUpDown.ValueChanged += new System.EventHandler(this.segColorWeightUpDown_ValueChanged);
      // 
      // label6
      // 
      this.label6.AutoSize = true;
      this.label6.Location = new System.Drawing.Point(6, 109);
      this.label6.Name = "label6";
      this.label6.Size = new System.Drawing.Size(77, 19);
      this.label6.TabIndex = 31;
      this.label6.Text = "colorWgt:";
      // 
      // segSaveButton
      // 
      this.segSaveButton.Location = new System.Drawing.Point(225, 163);
      this.segSaveButton.Name = "segSaveButton";
      this.segSaveButton.Size = new System.Drawing.Size(75, 26);
      this.segSaveButton.TabIndex = 30;
      this.segSaveButton.Text = "Save";
      this.segSaveButton.UseVisualStyleBackColor = true;
      this.segSaveButton.Click += new System.EventHandler(this.segSaveButton_Click);
      // 
      // showOBBsCheckBox
      // 
      this.showOBBsCheckBox.AutoSize = true;
      this.showOBBsCheckBox.Location = new System.Drawing.Point(79, 27);
      this.showOBBsCheckBox.Name = "showOBBsCheckBox";
      this.showOBBsCheckBox.Size = new System.Drawing.Size(63, 23);
      this.showOBBsCheckBox.TabIndex = 29;
      this.showOBBsCheckBox.Text = "OBBs";
      this.showOBBsCheckBox.UseVisualStyleBackColor = true;
      this.showOBBsCheckBox.CheckedChanged += new System.EventHandler(this.showOBBsCheckBox_CheckedChanged);
      // 
      // minVertsUpDown
      // 
      this.minVertsUpDown.Location = new System.Drawing.Point(79, 79);
      this.minVertsUpDown.Maximum = new decimal(new int[] {
            10000,
            0,
            0,
            0});
      this.minVertsUpDown.Minimum = new decimal(new int[] {
            1,
            0,
            0,
            0});
      this.minVertsUpDown.Name = "minVertsUpDown";
      this.minVertsUpDown.Size = new System.Drawing.Size(61, 27);
      this.minVertsUpDown.TabIndex = 28;
      this.minVertsUpDown.Value = new decimal(new int[] {
            200,
            0,
            0,
            0});
      this.minVertsUpDown.ValueChanged += new System.EventHandler(this.minVertsUpDown_ValueChanged);
      // 
      // label5
      // 
      this.label5.AutoSize = true;
      this.label5.Location = new System.Drawing.Point(6, 81);
      this.label5.Name = "label5";
      this.label5.Size = new System.Drawing.Size(73, 19);
      this.label5.TabIndex = 27;
      this.label5.Text = "minVerts:";
      // 
      // segKthreshUpDown
      // 
      this.segKthreshUpDown.DecimalPlaces = 3;
      this.segKthreshUpDown.Increment = new decimal(new int[] {
            1,
            0,
            0,
            262144});
      this.segKthreshUpDown.Location = new System.Drawing.Point(79, 51);
      this.segKthreshUpDown.Maximum = new decimal(new int[] {
            10,
            0,
            0,
            0});
      this.segKthreshUpDown.Minimum = new decimal(new int[] {
            1,
            0,
            0,
            262144});
      this.segKthreshUpDown.Name = "segKthreshUpDown";
      this.segKthreshUpDown.Size = new System.Drawing.Size(61, 27);
      this.segKthreshUpDown.TabIndex = 26;
      this.segKthreshUpDown.Value = new decimal(new int[] {
            1,
            0,
            0,
            196608});
      this.segKthreshUpDown.ValueChanged += new System.EventHandler(this.segKthreshUpDown_ValueChanged);
      // 
      // kThreshLabel
      // 
      this.kThreshLabel.AutoSize = true;
      this.kThreshLabel.Location = new System.Drawing.Point(6, 53);
      this.kThreshLabel.Name = "kThreshLabel";
      this.kThreshLabel.Size = new System.Drawing.Size(67, 19);
      this.kThreshLabel.TabIndex = 25;
      this.kThreshLabel.Text = "kThresh:";
      // 
      // showSegmentationCheckbox
      // 
      this.showSegmentationCheckbox.AutoSize = true;
      this.showSegmentationCheckbox.Location = new System.Drawing.Point(10, 27);
      this.showSegmentationCheckbox.Name = "showSegmentationCheckbox";
      this.showSegmentationCheckbox.Size = new System.Drawing.Size(70, 23);
      this.showSegmentationCheckbox.TabIndex = 24;
      this.showSegmentationCheckbox.Text = "Colors";
      this.showSegmentationCheckbox.UseVisualStyleBackColor = true;
      this.showSegmentationCheckbox.CheckedChanged += new System.EventHandler(this.showSegmentationCheckbox_CheckedChanged);
      // 
      // timerInitialize
      // 
      this.timerInitialize.Enabled = true;
      this.timerInitialize.Tick += new System.EventHandler(this.timerInitialize_Tick);
      // 
      // timerDisconnectCheck
      // 
      this.timerDisconnectCheck.Enabled = true;
      this.timerDisconnectCheck.Interval = 500;
      this.timerDisconnectCheck.Tick += new System.EventHandler(this.timerDisconnectCheck_Tick);
      // 
      // timeScrollerA
      // 
      this.timeScrollerA.Location = new System.Drawing.Point(-3, 69);
      this.timeScrollerA.Maximum = 300000;
      this.timeScrollerA.Name = "timeScrollerA";
      this.timeScrollerA.Size = new System.Drawing.Size(813, 15);
      this.timeScrollerA.TabIndex = 0;
      this.timeScrollerA.Scroll += new System.Windows.Forms.ScrollEventHandler(this.timeScroller_Scroll);
      // 
      // label2
      // 
      this.label2.AutoSize = true;
      this.label2.Location = new System.Drawing.Point(6, 26);
      this.label2.Name = "label2";
      this.label2.Size = new System.Drawing.Size(104, 19);
      this.label2.TabIndex = 7;
      this.label2.Text = "TimeWindow:";
      // 
      // CameraLabel
      // 
      this.CameraLabel.AutoSize = true;
      this.CameraLabel.Location = new System.Drawing.Point(1, 552);
      this.CameraLabel.Name = "CameraLabel";
      this.CameraLabel.Size = new System.Drawing.Size(60, 19);
      this.CameraLabel.TabIndex = 10;
      this.CameraLabel.Text = "Camera";
      // 
      // cameraUpDown
      // 
      this.cameraUpDown.Location = new System.Drawing.Point(64, 547);
      this.cameraUpDown.Name = "cameraUpDown";
      this.cameraUpDown.Size = new System.Drawing.Size(35, 27);
      this.cameraUpDown.TabIndex = 11;
      this.cameraUpDown.ValueChanged += new System.EventHandler(this.cameraUpDown_ValueChanged);
      // 
      // integrationWindowUpDown
      // 
      this.integrationWindowUpDown.DecimalPlaces = 1;
      this.integrationWindowUpDown.Increment = new decimal(new int[] {
            1,
            0,
            0,
            65536});
      this.integrationWindowUpDown.Location = new System.Drawing.Point(108, 20);
      this.integrationWindowUpDown.Minimum = new decimal(new int[] {
            1,
            0,
            0,
            65536});
      this.integrationWindowUpDown.Name = "integrationWindowUpDown";
      this.integrationWindowUpDown.Size = new System.Drawing.Size(61, 27);
      this.integrationWindowUpDown.TabIndex = 15;
      this.integrationWindowUpDown.Value = new decimal(new int[] {
            5,
            0,
            0,
            65536});
      this.integrationWindowUpDown.ValueChanged += new System.EventHandler(this.integrationWindowUpDown_ValueChanged_1);
      // 
      // jointIndexUpDown
      // 
      this.jointIndexUpDown.Location = new System.Drawing.Point(167, 547);
      this.jointIndexUpDown.Maximum = new decimal(new int[] {
            24,
            0,
            0,
            0});
      this.jointIndexUpDown.Name = "jointIndexUpDown";
      this.jointIndexUpDown.Size = new System.Drawing.Size(35, 27);
      this.jointIndexUpDown.TabIndex = 16;
      this.jointIndexUpDown.ValueChanged += new System.EventHandler(this.jointIndexUpDown_ValueChanged);
      // 
      // label4
      // 
      this.label4.AutoSize = true;
      this.label4.Location = new System.Drawing.Point(116, 552);
      this.label4.Name = "label4";
      this.label4.Size = new System.Drawing.Size(46, 19);
      this.label4.TabIndex = 17;
      this.label4.Text = "iJoint";
      // 
      // actRlabel
      // 
      this.actRlabel.AutoSize = true;
      this.actRlabel.Location = new System.Drawing.Point(6, 73);
      this.actRlabel.Name = "actRlabel";
      this.actRlabel.Size = new System.Drawing.Size(92, 19);
      this.actRlabel.TabIndex = 18;
      this.actRlabel.Text = "ActivRadius:";
      // 
      // activationRadiusUpDown
      // 
      this.activationRadiusUpDown.DecimalPlaces = 2;
      this.activationRadiusUpDown.Increment = new decimal(new int[] {
            1,
            0,
            0,
            131072});
      this.activationRadiusUpDown.Location = new System.Drawing.Point(102, 71);
      this.activationRadiusUpDown.Maximum = new decimal(new int[] {
            1,
            0,
            0,
            0});
      this.activationRadiusUpDown.Minimum = new decimal(new int[] {
            1,
            0,
            0,
            131072});
      this.activationRadiusUpDown.Name = "activationRadiusUpDown";
      this.activationRadiusUpDown.Size = new System.Drawing.Size(61, 27);
      this.activationRadiusUpDown.TabIndex = 19;
      this.activationRadiusUpDown.Value = new decimal(new int[] {
            1,
            0,
            0,
            65536});
      this.activationRadiusUpDown.ValueChanged += new System.EventHandler(this.activationRadiusUpDown_ValueChanged);
      // 
      // activationMapCheckbox
      // 
      this.activationMapCheckbox.AutoSize = true;
      this.activationMapCheckbox.Location = new System.Drawing.Point(6, 47);
      this.activationMapCheckbox.Name = "activationMapCheckbox";
      this.activationMapCheckbox.Size = new System.Drawing.Size(98, 23);
      this.activationMapCheckbox.TabIndex = 20;
      this.activationMapCheckbox.Text = "ActivPaint";
      this.activationMapCheckbox.UseVisualStyleBackColor = true;
      this.activationMapCheckbox.CheckedChanged += new System.EventHandler(this.activationMapCheckbox_CheckedChanged);
      // 
      // showSkeletonCheckbox
      // 
      this.showSkeletonCheckbox.AutoSize = true;
      this.showSkeletonCheckbox.Checked = true;
      this.showSkeletonCheckbox.CheckState = System.Windows.Forms.CheckState.Checked;
      this.showSkeletonCheckbox.Location = new System.Drawing.Point(8, 26);
      this.showSkeletonCheckbox.Name = "showSkeletonCheckbox";
      this.showSkeletonCheckbox.Size = new System.Drawing.Size(56, 23);
      this.showSkeletonCheckbox.TabIndex = 21;
      this.showSkeletonCheckbox.Text = "Skel";
      this.showSkeletonCheckbox.UseVisualStyleBackColor = true;
      this.showSkeletonCheckbox.CheckedChanged += new System.EventHandler(this.showSkeletonCheckbox_CheckedChanged);
      // 
      // hideUnactivatedCheckbox
      // 
      this.hideUnactivatedCheckbox.AutoSize = true;
      this.hideUnactivatedCheckbox.Location = new System.Drawing.Point(104, 47);
      this.hideUnactivatedCheckbox.Name = "hideUnactivatedCheckbox";
      this.hideUnactivatedCheckbox.Size = new System.Drawing.Size(112, 23);
      this.hideUnactivatedCheckbox.TabIndex = 22;
      this.hideUnactivatedCheckbox.Text = "HideInactive";
      this.hideUnactivatedCheckbox.UseVisualStyleBackColor = true;
      this.hideUnactivatedCheckbox.CheckedChanged += new System.EventHandler(this.hideUnactivatedCheckbox_CheckedChanged);
      // 
      // showGazeCheckbox
      // 
      this.showGazeCheckbox.AutoSize = true;
      this.showGazeCheckbox.Location = new System.Drawing.Point(83, 28);
      this.showGazeCheckbox.Name = "showGazeCheckbox";
      this.showGazeCheckbox.Size = new System.Drawing.Size(60, 23);
      this.showGazeCheckbox.TabIndex = 23;
      this.showGazeCheckbox.Text = "Gaze";
      this.showGazeCheckbox.UseVisualStyleBackColor = true;
      this.showGazeCheckbox.CheckedChanged += new System.EventHandler(this.showGazeCheckbox_CheckedChanged);
      // 
      // showScanCheckBox
      // 
      this.showScanCheckBox.AutoSize = true;
      this.showScanCheckBox.Checked = true;
      this.showScanCheckBox.CheckState = System.Windows.Forms.CheckState.Checked;
      this.showScanCheckBox.Location = new System.Drawing.Point(8, 84);
      this.showScanCheckBox.Name = "showScanCheckBox";
      this.showScanCheckBox.Size = new System.Drawing.Size(60, 23);
      this.showScanCheckBox.TabIndex = 30;
      this.showScanCheckBox.Text = "Scan";
      this.showScanCheckBox.UseVisualStyleBackColor = true;
      this.showScanCheckBox.CheckedChanged += new System.EventHandler(this.showScanCheckBox_CheckedChanged);
      // 
      // showBodyPointCheckBox
      // 
      this.showBodyPointCheckBox.AutoSize = true;
      this.showBodyPointCheckBox.Location = new System.Drawing.Point(83, 55);
      this.showBodyPointCheckBox.Name = "showBodyPointCheckBox";
      this.showBodyPointCheckBox.Size = new System.Drawing.Size(75, 23);
      this.showBodyPointCheckBox.TabIndex = 31;
      this.showBodyPointCheckBox.Text = "BdyPts";
      this.showBodyPointCheckBox.UseVisualStyleBackColor = true;
      this.showBodyPointCheckBox.CheckedChanged += new System.EventHandler(this.showBodyPointCheckBox_CheckedChanged);
      // 
      // showInteractionButton
      // 
      this.showInteractionButton.AutoSize = true;
      this.showInteractionButton.Checked = true;
      this.showInteractionButton.CheckState = System.Windows.Forms.CheckState.Checked;
      this.showInteractionButton.Location = new System.Drawing.Point(104, 22);
      this.showInteractionButton.Name = "showInteractionButton";
      this.showInteractionButton.Size = new System.Drawing.Size(116, 23);
      this.showInteractionButton.TabIndex = 38;
      this.showInteractionButton.Text = "ContactOBBs";
      this.showInteractionButton.UseVisualStyleBackColor = true;
      this.showInteractionButton.CheckedChanged += new System.EventHandler(this.showInteractionButton_CheckedChanged);
      // 
      // timeScrollerB
      // 
      this.timeScrollerB.Location = new System.Drawing.Point(-3, 84);
      this.timeScrollerB.Maximum = 300000;
      this.timeScrollerB.Name = "timeScrollerB";
      this.timeScrollerB.Size = new System.Drawing.Size(813, 15);
      this.timeScrollerB.TabIndex = 1;
      this.timeScrollerB.Scroll += new System.Windows.Forms.ScrollEventHandler(this.timeScrollerB_Scroll);
      // 
      // sliderPicBar
      // 
      this.sliderPicBar.Location = new System.Drawing.Point(8, 56);
      this.sliderPicBar.Name = "sliderPicBar";
      this.sliderPicBar.Size = new System.Drawing.Size(793, 10);
      this.sliderPicBar.TabIndex = 40;
      this.sliderPicBar.TabStop = false;
      // 
      // showColorFramesCheckbox
      // 
      this.showColorFramesCheckbox.AutoSize = true;
      this.showColorFramesCheckbox.Location = new System.Drawing.Point(3, 444);
      this.showColorFramesCheckbox.Name = "showColorFramesCheckbox";
      this.showColorFramesCheckbox.Size = new System.Drawing.Size(112, 23);
      this.showColorFramesCheckbox.TabIndex = 41;
      this.showColorFramesCheckbox.Text = "ColorFrames";
      this.showColorFramesCheckbox.UseVisualStyleBackColor = true;
      this.showColorFramesCheckbox.CheckedChanged += new System.EventHandler(this.showColorFramesCheckbox_CheckedChanged);
      // 
      // depthFramesCheckbox
      // 
      this.depthFramesCheckbox.AutoSize = true;
      this.depthFramesCheckbox.Location = new System.Drawing.Point(111, 444);
      this.depthFramesCheckbox.Name = "depthFramesCheckbox";
      this.depthFramesCheckbox.Size = new System.Drawing.Size(118, 23);
      this.depthFramesCheckbox.TabIndex = 42;
      this.depthFramesCheckbox.Text = "DepthFrames";
      this.depthFramesCheckbox.UseVisualStyleBackColor = true;
      this.depthFramesCheckbox.CheckedChanged += new System.EventHandler(this.depthFramesCheckbox_CheckedChanged);
      // 
      // groupBox2
      // 
      this.groupBox2.Controls.Add(this.paintAllJointsCheckbox);
      this.groupBox2.Controls.Add(this.showInteractionButton);
      this.groupBox2.Controls.Add(this.showPIGSimCheckbox);
      this.groupBox2.Controls.Add(this.activationMapCheckbox);
      this.groupBox2.Controls.Add(this.showKNNIGSimCheckbox);
      this.groupBox2.Controls.Add(this.hideUnactivatedCheckbox);
      this.groupBox2.Controls.Add(this.actRlabel);
      this.groupBox2.Controls.Add(this.activationRadiusUpDown);
      this.groupBox2.Location = new System.Drawing.Point(-1, 187);
      this.groupBox2.Name = "groupBox2";
      this.groupBox2.Size = new System.Drawing.Size(222, 138);
      this.groupBox2.TabIndex = 43;
      this.groupBox2.TabStop = false;
      this.groupBox2.Text = "Interactions";
      // 
      // paintAllJointsCheckbox
      // 
      this.paintAllJointsCheckbox.AutoSize = true;
      this.paintAllJointsCheckbox.Location = new System.Drawing.Point(6, 22);
      this.paintAllJointsCheckbox.Name = "paintAllJointsCheckbox";
      this.paintAllJointsCheckbox.Size = new System.Drawing.Size(85, 23);
      this.paintAllJointsCheckbox.TabIndex = 40;
      this.paintAllJointsCheckbox.Text = "AllJoints";
      this.paintAllJointsCheckbox.UseVisualStyleBackColor = true;
      this.paintAllJointsCheckbox.CheckedChanged += new System.EventHandler(this.paintAllJointsCheckbox_CheckedChanged);
      // 
      // showPIGSimCheckbox
      // 
      this.showPIGSimCheckbox.AutoSize = true;
      this.showPIGSimCheckbox.Location = new System.Drawing.Point(109, 109);
      this.showPIGSimCheckbox.Name = "showPIGSimCheckbox";
      this.showPIGSimCheckbox.Size = new System.Drawing.Size(76, 23);
      this.showPIGSimCheckbox.TabIndex = 54;
      this.showPIGSimCheckbox.Text = "PIGSim";
      this.showPIGSimCheckbox.UseVisualStyleBackColor = true;
      this.showPIGSimCheckbox.CheckedChanged += new System.EventHandler(this.showPIGSimCheckbox_CheckedChanged);
      // 
      // showKNNIGSimCheckbox
      // 
      this.showKNNIGSimCheckbox.AutoSize = true;
      this.showKNNIGSimCheckbox.Location = new System.Drawing.Point(5, 109);
      this.showKNNIGSimCheckbox.Name = "showKNNIGSimCheckbox";
      this.showKNNIGSimCheckbox.Size = new System.Drawing.Size(98, 23);
      this.showKNNIGSimCheckbox.TabIndex = 53;
      this.showKNNIGSimCheckbox.Text = "KNNIGSim";
      this.showKNNIGSimCheckbox.UseVisualStyleBackColor = true;
      this.showKNNIGSimCheckbox.CheckedChanged += new System.EventHandler(this.showKNNIGSimCheckbox_CheckedChanged);
      // 
      // groupBox3
      // 
      this.groupBox3.Controls.Add(this.integrationWindowUpDown);
      this.groupBox3.Controls.Add(this.label2);
      this.groupBox3.Location = new System.Drawing.Point(-1, 102);
      this.groupBox3.Name = "groupBox3";
      this.groupBox3.Size = new System.Drawing.Size(222, 79);
      this.groupBox3.TabIndex = 44;
      this.groupBox3.TabStop = false;
      this.groupBox3.Text = "Integration";
      // 
      // annotationBox
      // 
      this.annotationBox.Controls.Add(this.loadAnnotationButton);
      this.annotationBox.Controls.Add(this.annDeleteButton);
      this.annotationBox.Controls.Add(this.playRangeButton);
      this.annotationBox.Controls.Add(this.annotationTextbox);
      this.annotationBox.Controls.Add(this.annotationDumpButton);
      this.annotationBox.Controls.Add(this.annotationPushButton);
      this.annotationBox.Controls.Add(this.annotationsBox);
      this.annotationBox.Location = new System.Drawing.Point(536, 102);
      this.annotationBox.Name = "annotationBox";
      this.annotationBox.Size = new System.Drawing.Size(246, 320);
      this.annotationBox.TabIndex = 45;
      this.annotationBox.TabStop = false;
      this.annotationBox.Text = "Annotation";
      // 
      // loadAnnotationButton
      // 
      this.loadAnnotationButton.Location = new System.Drawing.Point(165, 248);
      this.loadAnnotationButton.Name = "loadAnnotationButton";
      this.loadAnnotationButton.Size = new System.Drawing.Size(75, 27);
      this.loadAnnotationButton.TabIndex = 48;
      this.loadAnnotationButton.Text = "Load";
      this.loadAnnotationButton.UseVisualStyleBackColor = true;
      this.loadAnnotationButton.Click += new System.EventHandler(this.loadAnnotationButton_Click);
      // 
      // annDeleteButton
      // 
      this.annDeleteButton.Location = new System.Drawing.Point(86, 281);
      this.annDeleteButton.Name = "annDeleteButton";
      this.annDeleteButton.Size = new System.Drawing.Size(64, 27);
      this.annDeleteButton.TabIndex = 47;
      this.annDeleteButton.Text = "Delete";
      this.annDeleteButton.UseVisualStyleBackColor = true;
      this.annDeleteButton.Click += new System.EventHandler(this.annDeleteButton_Click);
      // 
      // playRangeButton
      // 
      this.playRangeButton.Appearance = System.Windows.Forms.Appearance.Button;
      this.playRangeButton.Location = new System.Drawing.Point(6, 248);
      this.playRangeButton.Name = "playRangeButton";
      this.playRangeButton.Size = new System.Drawing.Size(60, 27);
      this.playRangeButton.TabIndex = 42;
      this.playRangeButton.Text = "Play";
      this.playRangeButton.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
      this.playRangeButton.UseVisualStyleBackColor = true;
      this.playRangeButton.CheckedChanged += new System.EventHandler(this.playRangeButton_CheckedChanged);
      // 
      // annotationTextbox
      // 
      this.annotationTextbox.Location = new System.Drawing.Point(6, 215);
      this.annotationTextbox.Name = "annotationTextbox";
      this.annotationTextbox.Size = new System.Drawing.Size(234, 27);
      this.annotationTextbox.TabIndex = 0;
      this.annotationTextbox.TextChanged += new System.EventHandler(this.annotationTextbox_TextChanged);
      // 
      // annotationDumpButton
      // 
      this.annotationDumpButton.Location = new System.Drawing.Point(165, 281);
      this.annotationDumpButton.Name = "annotationDumpButton";
      this.annotationDumpButton.Size = new System.Drawing.Size(75, 27);
      this.annotationDumpButton.TabIndex = 40;
      this.annotationDumpButton.Text = "Dump";
      this.annotationDumpButton.UseVisualStyleBackColor = true;
      this.annotationDumpButton.Click += new System.EventHandler(this.annotationDumpButton_Click);
      // 
      // annotationPushButton
      // 
      this.annotationPushButton.Location = new System.Drawing.Point(6, 281);
      this.annotationPushButton.Name = "annotationPushButton";
      this.annotationPushButton.Size = new System.Drawing.Size(60, 27);
      this.annotationPushButton.TabIndex = 39;
      this.annotationPushButton.Text = "Push";
      this.annotationPushButton.UseVisualStyleBackColor = true;
      this.annotationPushButton.Click += new System.EventHandler(this.annotationPushButton_Click);
      // 
      // annotationsBox
      // 
      this.annotationsBox.FormattingEnabled = true;
      this.annotationsBox.ItemHeight = 19;
      this.annotationsBox.Location = new System.Drawing.Point(6, 27);
      this.annotationsBox.Name = "annotationsBox";
      this.annotationsBox.Size = new System.Drawing.Size(234, 175);
      this.annotationsBox.TabIndex = 46;
      this.annotationsBox.SelectedIndexChanged += new System.EventHandler(this.annotationsBox_SelectedIndexChanged);
      // 
      // groupBox4
      // 
      this.groupBox4.Controls.Add(this.nextISetButton);
      this.groupBox4.Controls.Add(this.testInteractionButton);
      this.groupBox4.Controls.Add(this.classifierTypeDropdown);
      this.groupBox4.Controls.Add(this.dumpImagesCheckbox);
      this.groupBox4.Controls.Add(this.testButton);
      this.groupBox4.Controls.Add(this.cvButton);
      this.groupBox4.Controls.Add(this.trainAllButton);
      this.groupBox4.Controls.Add(this.trainButton);
      this.groupBox4.Controls.Add(this.segPerJointClassifyButton);
      this.groupBox4.Controls.Add(this.recomputeHeatmapButton);
      this.groupBox4.Controls.Add(this.useSingleSkelForHeatmapCheckbox);
      this.groupBox4.Controls.Add(this.interactionHeatmapShowAngle);
      this.groupBox4.Controls.Add(this.interactionHeatmapUseMax);
      this.groupBox4.Controls.Add(this.showSegVerbProbButton);
      this.groupBox4.Controls.Add(this.checkBoxShowClusterAssignment);
      this.groupBox4.Controls.Add(this.showInteractionHeatmap);
      this.groupBox4.Controls.Add(this.nextPoseButton);
      this.groupBox4.Controls.Add(this.createDBbutton);
      this.groupBox4.Location = new System.Drawing.Point(224, 300);
      this.groupBox4.Name = "groupBox4";
      this.groupBox4.Size = new System.Drawing.Size(303, 315);
      this.groupBox4.TabIndex = 46;
      this.groupBox4.TabStop = false;
      this.groupBox4.Text = "SceneGrok";
      // 
      // nextISetButton
      // 
      this.nextISetButton.Location = new System.Drawing.Point(108, 15);
      this.nextISetButton.Name = "nextISetButton";
      this.nextISetButton.Size = new System.Drawing.Size(82, 29);
      this.nextISetButton.TabIndex = 63;
      this.nextISetButton.Text = "NextISet";
      this.nextISetButton.UseVisualStyleBackColor = true;
      this.nextISetButton.Click += new System.EventHandler(this.nextISetButton_Click);
      // 
      // testInteractionButton
      // 
      this.testInteractionButton.Location = new System.Drawing.Point(113, 137);
      this.testInteractionButton.Name = "testInteractionButton";
      this.testInteractionButton.Size = new System.Drawing.Size(128, 29);
      this.testInteractionButton.TabIndex = 62;
      this.testInteractionButton.Text = "TestInteraction";
      this.testInteractionButton.UseVisualStyleBackColor = true;
      this.testInteractionButton.Click += new System.EventHandler(this.testInteractionButton_Click);
      // 
      // classifierTypeDropdown
      // 
      this.classifierTypeDropdown.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
      this.classifierTypeDropdown.FormattingEnabled = true;
      this.classifierTypeDropdown.Items.AddRange(new object[] {
            "IGKNNSim",
            "segmentJointsAggr",
            "segmentCentroidActivation",
            "segmentJointsLWAggr"});
      this.classifierTypeDropdown.Location = new System.Drawing.Point(7, 95);
      this.classifierTypeDropdown.Name = "classifierTypeDropdown";
      this.classifierTypeDropdown.Size = new System.Drawing.Size(183, 27);
      this.classifierTypeDropdown.TabIndex = 61;
      this.classifierTypeDropdown.SelectedIndexChanged += new System.EventHandler(this.classifierTypeDropdown_SelectedIndexChanged);
      // 
      // dumpImagesCheckbox
      // 
      this.dumpImagesCheckbox.AutoSize = true;
      this.dumpImagesCheckbox.Location = new System.Drawing.Point(197, 97);
      this.dumpImagesCheckbox.Name = "dumpImagesCheckbox";
      this.dumpImagesCheckbox.Size = new System.Drawing.Size(100, 23);
      this.dumpImagesCheckbox.TabIndex = 54;
      this.dumpImagesCheckbox.Text = "DumpImgs";
      this.dumpImagesCheckbox.UseVisualStyleBackColor = true;
      this.dumpImagesCheckbox.CheckedChanged += new System.EventHandler(this.dumpImagesCheckbox_CheckedChanged);
      // 
      // testButton
      // 
      this.testButton.Location = new System.Drawing.Point(182, 60);
      this.testButton.Name = "testButton";
      this.testButton.Size = new System.Drawing.Size(57, 29);
      this.testButton.TabIndex = 53;
      this.testButton.Text = "Test";
      this.testButton.UseVisualStyleBackColor = true;
      this.testButton.Click += new System.EventHandler(this.testButton_Click);
      // 
      // cvButton
      // 
      this.cvButton.Location = new System.Drawing.Point(245, 60);
      this.cvButton.Name = "cvButton";
      this.cvButton.Size = new System.Drawing.Size(36, 29);
      this.cvButton.TabIndex = 52;
      this.cvButton.Text = "CV";
      this.cvButton.UseVisualStyleBackColor = true;
      this.cvButton.Click += new System.EventHandler(this.cvButton_Click);
      // 
      // trainAllButton
      // 
      this.trainAllButton.Location = new System.Drawing.Point(96, 60);
      this.trainAllButton.Name = "trainAllButton";
      this.trainAllButton.Size = new System.Drawing.Size(82, 29);
      this.trainAllButton.TabIndex = 51;
      this.trainAllButton.Text = "TrainAll";
      this.trainAllButton.UseVisualStyleBackColor = true;
      this.trainAllButton.Click += new System.EventHandler(this.trainAllButton_Click);
      // 
      // trainButton
      // 
      this.trainButton.Location = new System.Drawing.Point(8, 60);
      this.trainButton.Name = "trainButton";
      this.trainButton.Size = new System.Drawing.Size(82, 29);
      this.trainButton.TabIndex = 50;
      this.trainButton.Text = "Train";
      this.trainButton.UseVisualStyleBackColor = true;
      this.trainButton.Click += new System.EventHandler(this.trainButton_Click);
      // 
      // segPerJointClassifyButton
      // 
      this.segPerJointClassifyButton.Location = new System.Drawing.Point(7, 243);
      this.segPerJointClassifyButton.Name = "segPerJointClassifyButton";
      this.segPerJointClassifyButton.Size = new System.Drawing.Size(152, 26);
      this.segPerJointClassifyButton.TabIndex = 46;
      this.segPerJointClassifyButton.Text = "segPerJointClassify";
      this.segPerJointClassifyButton.UseVisualStyleBackColor = true;
      this.segPerJointClassifyButton.Click += new System.EventHandler(this.segPerJointClassifyButton_Click);
      // 
      // recomputeHeatmapButton
      // 
      this.recomputeHeatmapButton.Location = new System.Drawing.Point(8, 137);
      this.recomputeHeatmapButton.Name = "recomputeHeatmapButton";
      this.recomputeHeatmapButton.Size = new System.Drawing.Size(96, 29);
      this.recomputeHeatmapButton.TabIndex = 43;
      this.recomputeHeatmapButton.Text = "Recompute";
      this.recomputeHeatmapButton.UseVisualStyleBackColor = true;
      this.recomputeHeatmapButton.Click += new System.EventHandler(this.recomputeHeatmapButton_Click);
      // 
      // useSingleSkelForHeatmapCheckbox
      // 
      this.useSingleSkelForHeatmapCheckbox.AutoSize = true;
      this.useSingleSkelForHeatmapCheckbox.Checked = true;
      this.useSingleSkelForHeatmapCheckbox.CheckState = System.Windows.Forms.CheckState.Checked;
      this.useSingleSkelForHeatmapCheckbox.Location = new System.Drawing.Point(125, 214);
      this.useSingleSkelForHeatmapCheckbox.Name = "useSingleSkelForHeatmapCheckbox";
      this.useSingleSkelForHeatmapCheckbox.Size = new System.Drawing.Size(97, 23);
      this.useSingleSkelForHeatmapCheckbox.TabIndex = 49;
      this.useSingleSkelForHeatmapCheckbox.Text = "SingleSkel";
      this.useSingleSkelForHeatmapCheckbox.UseVisualStyleBackColor = true;
      this.useSingleSkelForHeatmapCheckbox.CheckedChanged += new System.EventHandler(this.useSingleSkelForHeatmapCheckbox_CheckedChanged);
      // 
      // interactionHeatmapShowAngle
      // 
      this.interactionHeatmapShowAngle.AutoSize = true;
      this.interactionHeatmapShowAngle.Checked = true;
      this.interactionHeatmapShowAngle.CheckState = System.Windows.Forms.CheckState.Checked;
      this.interactionHeatmapShowAngle.Location = new System.Drawing.Point(9, 214);
      this.interactionHeatmapShowAngle.Name = "interactionHeatmapShowAngle";
      this.interactionHeatmapShowAngle.Size = new System.Drawing.Size(105, 23);
      this.interactionHeatmapShowAngle.TabIndex = 47;
      this.interactionHeatmapShowAngle.Text = "ShowAngle";
      this.interactionHeatmapShowAngle.UseVisualStyleBackColor = true;
      this.interactionHeatmapShowAngle.CheckedChanged += new System.EventHandler(this.interactionHeatmapShowAngle_CheckedChanged);
      // 
      // interactionHeatmapUseMax
      // 
      this.interactionHeatmapUseMax.AutoSize = true;
      this.interactionHeatmapUseMax.Checked = true;
      this.interactionHeatmapUseMax.CheckState = System.Windows.Forms.CheckState.Checked;
      this.interactionHeatmapUseMax.Location = new System.Drawing.Point(9, 190);
      this.interactionHeatmapUseMax.Name = "interactionHeatmapUseMax";
      this.interactionHeatmapUseMax.Size = new System.Drawing.Size(67, 23);
      this.interactionHeatmapUseMax.TabIndex = 44;
      this.interactionHeatmapUseMax.Text = "Norm";
      this.interactionHeatmapUseMax.UseVisualStyleBackColor = true;
      this.interactionHeatmapUseMax.CheckedChanged += new System.EventHandler(this.interactionHeatmapUseMax_CheckedChanged);
      // 
      // showSegVerbProbButton
      // 
      this.showSegVerbProbButton.AutoSize = true;
      this.showSegVerbProbButton.Location = new System.Drawing.Point(125, 171);
      this.showSegVerbProbButton.Name = "showSegVerbProbButton";
      this.showSegVerbProbButton.Size = new System.Drawing.Size(116, 23);
      this.showSegVerbProbButton.TabIndex = 42;
      this.showSegVerbProbButton.Text = "SegVerbProb";
      this.showSegVerbProbButton.UseVisualStyleBackColor = true;
      this.showSegVerbProbButton.CheckedChanged += new System.EventHandler(this.showSegVerbProbButton_CheckedChanged);
      // 
      // checkBoxShowClusterAssignment
      // 
      this.checkBoxShowClusterAssignment.AutoSize = true;
      this.checkBoxShowClusterAssignment.Location = new System.Drawing.Point(125, 192);
      this.checkBoxShowClusterAssignment.Name = "checkBoxShowClusterAssignment";
      this.checkBoxShowClusterAssignment.Size = new System.Drawing.Size(104, 23);
      this.checkBoxShowClusterAssignment.TabIndex = 39;
      this.checkBoxShowClusterAssignment.Text = "ClustAssign";
      this.checkBoxShowClusterAssignment.UseVisualStyleBackColor = true;
      this.checkBoxShowClusterAssignment.CheckedChanged += new System.EventHandler(this.checkBoxShowClusterAssignment_CheckedChanged);
      // 
      // showInteractionHeatmap
      // 
      this.showInteractionHeatmap.AutoSize = true;
      this.showInteractionHeatmap.Location = new System.Drawing.Point(9, 168);
      this.showInteractionHeatmap.Name = "showInteractionHeatmap";
      this.showInteractionHeatmap.Size = new System.Drawing.Size(91, 23);
      this.showInteractionHeatmap.TabIndex = 39;
      this.showInteractionHeatmap.Text = "HeatMap";
      this.showInteractionHeatmap.UseVisualStyleBackColor = true;
      this.showInteractionHeatmap.CheckedChanged += new System.EventHandler(this.showInteractionHeatmap_CheckedChanged);
      // 
      // nextPoseButton
      // 
      this.nextPoseButton.Location = new System.Drawing.Point(196, 15);
      this.nextPoseButton.Name = "nextPoseButton";
      this.nextPoseButton.Size = new System.Drawing.Size(82, 29);
      this.nextPoseButton.TabIndex = 41;
      this.nextPoseButton.Text = "NextPose";
      this.nextPoseButton.UseVisualStyleBackColor = true;
      this.nextPoseButton.Click += new System.EventHandler(this.nextPoseButton_Click);
      // 
      // createDBbutton
      // 
      this.createDBbutton.Location = new System.Drawing.Point(6, 16);
      this.createDBbutton.Name = "createDBbutton";
      this.createDBbutton.Size = new System.Drawing.Size(96, 29);
      this.createDBbutton.TabIndex = 40;
      this.createDBbutton.Text = "CreateDb";
      this.createDBbutton.UseVisualStyleBackColor = true;
      this.createDBbutton.Click += new System.EventHandler(this.createDBbutton_Click);
      // 
      // nextRecordingButton
      // 
      this.nextRecordingButton.Location = new System.Drawing.Point(-1, 403);
      this.nextRecordingButton.Name = "nextRecordingButton";
      this.nextRecordingButton.Size = new System.Drawing.Size(129, 29);
      this.nextRecordingButton.TabIndex = 40;
      this.nextRecordingButton.Text = "Next Recording";
      this.nextRecordingButton.UseVisualStyleBackColor = true;
      this.nextRecordingButton.Click += new System.EventHandler(this.nextRecordingButton_Click);
      // 
      // recIdLabel
      // 
      this.recIdLabel.AutoEllipsis = true;
      this.recIdLabel.Location = new System.Drawing.Point(2, 358);
      this.recIdLabel.Name = "recIdLabel";
      this.recIdLabel.Size = new System.Drawing.Size(219, 19);
      this.recIdLabel.TabIndex = 47;
      this.recIdLabel.Text = "Recording:";
      // 
      // buttonHeatMesh
      // 
      this.buttonHeatMesh.Location = new System.Drawing.Point(-2, 511);
      this.buttonHeatMesh.Name = "buttonHeatMesh";
      this.buttonHeatMesh.Size = new System.Drawing.Size(101, 32);
      this.buttonHeatMesh.TabIndex = 49;
      this.buttonHeatMesh.Text = "DumpMesh";
      this.buttonHeatMesh.UseVisualStyleBackColor = true;
      this.buttonHeatMesh.Click += new System.EventHandler(this.buttonDumpMesh_Click);
      // 
      // textBoxLoad
      // 
      this.textBoxLoad.Location = new System.Drawing.Point(70, 478);
      this.textBoxLoad.Name = "textBoxLoad";
      this.textBoxLoad.Size = new System.Drawing.Size(82, 27);
      this.textBoxLoad.TabIndex = 50;
      this.textBoxLoad.Text = "gates400";
      // 
      // buttonLoad
      // 
      this.buttonLoad.Location = new System.Drawing.Point(-3, 478);
      this.buttonLoad.Name = "buttonLoad";
      this.buttonLoad.Size = new System.Drawing.Size(67, 27);
      this.buttonLoad.TabIndex = 51;
      this.buttonLoad.Text = "Load:";
      this.buttonLoad.UseVisualStyleBackColor = true;
      this.buttonLoad.Click += new System.EventHandler(this.buttonLoad_Click);
      // 
      // helpButton
      // 
      this.helpButton.ForeColor = System.Drawing.Color.Blue;
      this.helpButton.Location = new System.Drawing.Point(787, 4);
      this.helpButton.Name = "helpButton";
      this.helpButton.Size = new System.Drawing.Size(32, 22);
      this.helpButton.TabIndex = 52;
      this.helpButton.Text = "?";
      this.helpButton.UseVisualStyleBackColor = true;
      this.helpButton.Click += new System.EventHandler(this.helpButton_Click);
      // 
      // showInteractionFrameCheckbox
      // 
      this.showInteractionFrameCheckbox.AutoSize = true;
      this.showInteractionFrameCheckbox.Location = new System.Drawing.Point(8, 55);
      this.showInteractionFrameCheckbox.Name = "showInteractionFrameCheckbox";
      this.showInteractionFrameCheckbox.Size = new System.Drawing.Size(39, 23);
      this.showInteractionFrameCheckbox.TabIndex = 55;
      this.showInteractionFrameCheckbox.Text = "IF";
      this.showInteractionFrameCheckbox.UseVisualStyleBackColor = true;
      this.showInteractionFrameCheckbox.CheckedChanged += new System.EventHandler(this.showInteractionFrameCheckbox_CheckedChanged);
      // 
      // ModelsGroup
      // 
      this.ModelsGroup.Controls.Add(this.showModelInteractionMapCheckbox);
      this.ModelsGroup.Controls.Add(this.showModelsCheckbox);
      this.ModelsGroup.Controls.Add(this.modelLoadTypeDropdown);
      this.ModelsGroup.Controls.Add(this.showModelVoxelsCheckBox);
      this.ModelsGroup.Controls.Add(this.loadModelSegFromFileCheckBox);
      this.ModelsGroup.Controls.Add(this.showModelSegmentationCheckbox);
      this.ModelsGroup.Controls.Add(this.modelTextBoxLoad);
      this.ModelsGroup.Controls.Add(this.loadModelButton);
      this.ModelsGroup.Location = new System.Drawing.Point(-2, 593);
      this.ModelsGroup.Name = "ModelsGroup";
      this.ModelsGroup.Size = new System.Drawing.Size(803, 56);
      this.ModelsGroup.TabIndex = 56;
      this.ModelsGroup.TabStop = false;
      this.ModelsGroup.Text = "Models";
      // 
      // showModelInteractionMapCheckbox
      // 
      this.showModelInteractionMapCheckbox.AutoSize = true;
      this.showModelInteractionMapCheckbox.Checked = true;
      this.showModelInteractionMapCheckbox.CheckState = System.Windows.Forms.CheckState.Checked;
      this.showModelInteractionMapCheckbox.Location = new System.Drawing.Point(627, 29);
      this.showModelInteractionMapCheckbox.Name = "showModelInteractionMapCheckbox";
      this.showModelInteractionMapCheckbox.Size = new System.Drawing.Size(46, 23);
      this.showModelInteractionMapCheckbox.TabIndex = 63;
      this.showModelInteractionMapCheckbox.Text = "IM";
      this.showModelInteractionMapCheckbox.UseVisualStyleBackColor = true;
      this.showModelInteractionMapCheckbox.CheckedChanged += new System.EventHandler(this.showModelInteractionMapCheckbox_CheckedChanged);
      // 
      // showModelsCheckbox
      // 
      this.showModelsCheckbox.AutoSize = true;
      this.showModelsCheckbox.Location = new System.Drawing.Point(682, 29);
      this.showModelsCheckbox.Name = "showModelsCheckbox";
      this.showModelsCheckbox.Size = new System.Drawing.Size(78, 23);
      this.showModelsCheckbox.TabIndex = 57;
      this.showModelsCheckbox.Text = "Models";
      this.showModelsCheckbox.UseVisualStyleBackColor = true;
      this.showModelsCheckbox.CheckedChanged += new System.EventHandler(this.showModelsCheckbox_CheckedChanged);
      // 
      // modelLoadTypeDropdown
      // 
      this.modelLoadTypeDropdown.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
      this.modelLoadTypeDropdown.FormattingEnabled = true;
      this.modelLoadTypeDropdown.Items.AddRange(new object[] {
            "category",
            "id"});
      this.modelLoadTypeDropdown.Location = new System.Drawing.Point(210, 26);
      this.modelLoadTypeDropdown.Name = "modelLoadTypeDropdown";
      this.modelLoadTypeDropdown.Size = new System.Drawing.Size(137, 27);
      this.modelLoadTypeDropdown.TabIndex = 62;
      // 
      // showModelVoxelsCheckBox
      // 
      this.showModelVoxelsCheckBox.AutoSize = true;
      this.showModelVoxelsCheckBox.Location = new System.Drawing.Point(365, 28);
      this.showModelVoxelsCheckBox.Name = "showModelVoxelsCheckBox";
      this.showModelVoxelsCheckBox.Size = new System.Drawing.Size(70, 23);
      this.showModelVoxelsCheckBox.TabIndex = 62;
      this.showModelVoxelsCheckBox.Text = "Voxels";
      this.showModelVoxelsCheckBox.UseVisualStyleBackColor = true;
      this.showModelVoxelsCheckBox.CheckedChanged += new System.EventHandler(this.showModelVoxelsCheckBox_CheckedChanged);
      // 
      // loadModelSegFromFileCheckBox
      // 
      this.loadModelSegFromFileCheckBox.AutoSize = true;
      this.loadModelSegFromFileCheckBox.Checked = true;
      this.loadModelSegFromFileCheckBox.CheckState = System.Windows.Forms.CheckState.Checked;
      this.loadModelSegFromFileCheckBox.Location = new System.Drawing.Point(536, 28);
      this.loadModelSegFromFileCheckBox.Name = "loadModelSegFromFileCheckBox";
      this.loadModelSegFromFileCheckBox.Size = new System.Drawing.Size(91, 23);
      this.loadModelSegFromFileCheckBox.TabIndex = 61;
      this.loadModelSegFromFileCheckBox.Text = "LoadSegs";
      this.loadModelSegFromFileCheckBox.UseVisualStyleBackColor = true;
      this.loadModelSegFromFileCheckBox.CheckedChanged += new System.EventHandler(this.loadModelSegFromFileCheckBox_CheckedChanged);
      // 
      // showModelSegmentationCheckbox
      // 
      this.showModelSegmentationCheckbox.AutoSize = true;
      this.showModelSegmentationCheckbox.Location = new System.Drawing.Point(436, 29);
      this.showModelSegmentationCheckbox.Name = "showModelSegmentationCheckbox";
      this.showModelSegmentationCheckbox.Size = new System.Drawing.Size(94, 23);
      this.showModelSegmentationCheckbox.TabIndex = 60;
      this.showModelSegmentationCheckbox.Text = "SegColors";
      this.showModelSegmentationCheckbox.UseVisualStyleBackColor = true;
      this.showModelSegmentationCheckbox.CheckedChanged += new System.EventHandler(this.showModelSegmentationCheckbox_CheckedChanged);
      // 
      // modelTextBoxLoad
      // 
      this.modelTextBoxLoad.Location = new System.Drawing.Point(85, 27);
      this.modelTextBoxLoad.Name = "modelTextBoxLoad";
      this.modelTextBoxLoad.Size = new System.Drawing.Size(119, 27);
      this.modelTextBoxLoad.TabIndex = 58;
      this.modelTextBoxLoad.Text = "Chair";
      // 
      // loadModelButton
      // 
      this.loadModelButton.Location = new System.Drawing.Point(9, 26);
      this.loadModelButton.Name = "loadModelButton";
      this.loadModelButton.Size = new System.Drawing.Size(67, 27);
      this.loadModelButton.TabIndex = 57;
      this.loadModelButton.Text = "Load:";
      this.loadModelButton.UseVisualStyleBackColor = true;
      this.loadModelButton.Click += new System.EventHandler(this.loadModelButton_Click);
      // 
      // recDurationLabel
      // 
      this.recDurationLabel.AutoSize = true;
      this.recDurationLabel.Location = new System.Drawing.Point(1, 381);
      this.recDurationLabel.Name = "recDurationLabel";
      this.recDurationLabel.Size = new System.Drawing.Size(101, 19);
      this.recDurationLabel.TabIndex = 16;
      this.recDurationLabel.Text = "RecDuration: ";
      // 
      // visualizationGroupBox
      // 
      this.visualizationGroupBox.Controls.Add(this.showScanLabeledVoxelsCheckBox);
      this.visualizationGroupBox.Controls.Add(this.showScanVoxelsCheckBox);
      this.visualizationGroupBox.Controls.Add(this.showInteractionFrameCheckbox);
      this.visualizationGroupBox.Controls.Add(this.showBodyPointCheckBox);
      this.visualizationGroupBox.Controls.Add(this.showScanCheckBox);
      this.visualizationGroupBox.Controls.Add(this.showGazeCheckbox);
      this.visualizationGroupBox.Controls.Add(this.showSkeletonCheckbox);
      this.visualizationGroupBox.Location = new System.Drawing.Point(539, 437);
      this.visualizationGroupBox.Name = "visualizationGroupBox";
      this.visualizationGroupBox.Size = new System.Drawing.Size(239, 159);
      this.visualizationGroupBox.TabIndex = 58;
      this.visualizationGroupBox.TabStop = false;
      this.visualizationGroupBox.Text = "Visualization";
      // 
      // showScanLabeledVoxelsCheckBox
      // 
      this.showScanLabeledVoxelsCheckBox.AutoSize = true;
      this.showScanLabeledVoxelsCheckBox.Location = new System.Drawing.Point(83, 111);
      this.showScanLabeledVoxelsCheckBox.Name = "showScanLabeledVoxelsCheckBox";
      this.showScanLabeledVoxelsCheckBox.Size = new System.Drawing.Size(123, 23);
      this.showScanLabeledVoxelsCheckBox.TabIndex = 57;
      this.showScanLabeledVoxelsCheckBox.Text = "LabeledVoxels";
      this.showScanLabeledVoxelsCheckBox.UseVisualStyleBackColor = true;
      this.showScanLabeledVoxelsCheckBox.CheckedChanged += new System.EventHandler(this.showScanLabeledVoxels_CheckedChanged);
      // 
      // showScanVoxelsCheckBox
      // 
      this.showScanVoxelsCheckBox.AutoSize = true;
      this.showScanVoxelsCheckBox.Location = new System.Drawing.Point(6, 111);
      this.showScanVoxelsCheckBox.Name = "showScanVoxelsCheckBox";
      this.showScanVoxelsCheckBox.Size = new System.Drawing.Size(70, 23);
      this.showScanVoxelsCheckBox.TabIndex = 56;
      this.showScanVoxelsCheckBox.Text = "Voxels";
      this.showScanVoxelsCheckBox.UseVisualStyleBackColor = true;
      this.showScanVoxelsCheckBox.CheckedChanged += new System.EventHandler(this.showScanVoxelsCheckBox_CheckedChanged);
      // 
      // modeGroupbox
      // 
      this.modeGroupbox.Controls.Add(this.showAllInteractionSetsCheckBox);
      this.modeGroupbox.Controls.Add(this.poseModeRadio);
      this.modeGroupbox.Controls.Add(this.replayModeRadio);
      this.modeGroupbox.Location = new System.Drawing.Point(11, 4);
      this.modeGroupbox.Name = "modeGroupbox";
      this.modeGroupbox.Size = new System.Drawing.Size(325, 46);
      this.modeGroupbox.TabIndex = 59;
      this.modeGroupbox.TabStop = false;
      this.modeGroupbox.Text = "Mode";
      // 
      // showAllInteractionSetsCheckBox
      // 
      this.showAllInteractionSetsCheckBox.AutoSize = true;
      this.showAllInteractionSetsCheckBox.Location = new System.Drawing.Point(260, 24);
      this.showAllInteractionSetsCheckBox.Name = "showAllInteractionSetsCheckBox";
      this.showAllInteractionSetsCheckBox.Size = new System.Drawing.Size(46, 23);
      this.showAllInteractionSetsCheckBox.TabIndex = 56;
      this.showAllInteractionSetsCheckBox.Text = "All";
      this.showAllInteractionSetsCheckBox.UseVisualStyleBackColor = true;
      this.showAllInteractionSetsCheckBox.CheckedChanged += new System.EventHandler(this.showAllInteractionSetsCheckBox_CheckedChanged);
      // 
      // poseModeRadio
      // 
      this.poseModeRadio.AutoSize = true;
      this.poseModeRadio.Location = new System.Drawing.Point(84, 23);
      this.poseModeRadio.Name = "poseModeRadio";
      this.poseModeRadio.Size = new System.Drawing.Size(59, 23);
      this.poseModeRadio.TabIndex = 1;
      this.poseModeRadio.Text = "Pose";
      this.poseModeRadio.UseVisualStyleBackColor = true;
      this.poseModeRadio.CheckedChanged += new System.EventHandler(this.modeRadio_CheckedChanged);
      // 
      // replayModeRadio
      // 
      this.replayModeRadio.AutoSize = true;
      this.replayModeRadio.Checked = true;
      this.replayModeRadio.Location = new System.Drawing.Point(7, 23);
      this.replayModeRadio.Name = "replayModeRadio";
      this.replayModeRadio.Size = new System.Drawing.Size(73, 23);
      this.replayModeRadio.TabIndex = 0;
      this.replayModeRadio.TabStop = true;
      this.replayModeRadio.Text = "Replay";
      this.replayModeRadio.UseVisualStyleBackColor = true;
      this.replayModeRadio.CheckedChanged += new System.EventHandler(this.modeRadio_CheckedChanged);
      // 
      // interactionDropdown
      // 
      this.interactionDropdown.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
      this.interactionDropdown.FormattingEnabled = true;
      this.interactionDropdown.Location = new System.Drawing.Point(347, 23);
      this.interactionDropdown.Name = "interactionDropdown";
      this.interactionDropdown.Size = new System.Drawing.Size(353, 27);
      this.interactionDropdown.TabIndex = 55;
      this.interactionDropdown.SelectedIndexChanged += new System.EventHandler(this.interactionDropdown_SelectedIndexChanged);
      // 
      // interactionTypeDropdown
      // 
      this.interactionTypeDropdown.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
      this.interactionTypeDropdown.FormattingEnabled = true;
      this.interactionTypeDropdown.Items.AddRange(new object[] {
            "composite",
            "verbNoun",
            "verb"});
      this.interactionTypeDropdown.Location = new System.Drawing.Point(701, 23);
      this.interactionTypeDropdown.Name = "interactionTypeDropdown";
      this.interactionTypeDropdown.Size = new System.Drawing.Size(118, 27);
      this.interactionTypeDropdown.TabIndex = 60;
      this.interactionTypeDropdown.SelectedIndexChanged += new System.EventHandler(this.interactionTypeDropdown_SelectedIndexChanged);
      // 
      // UIWindow
      // 
      this.AcceptButton = this.annotationPushButton;
      this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 19F);
      this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
      this.AutoScroll = true;
      this.ClientSize = new System.Drawing.Size(834, 651);
      this.Controls.Add(this.helpButton);
      this.Controls.Add(this.interactionTypeDropdown);
      this.Controls.Add(this.interactionDropdown);
      this.Controls.Add(this.modeGroupbox);
      this.Controls.Add(this.visualizationGroupBox);
      this.Controls.Add(this.ModelsGroup);
      this.Controls.Add(this.buttonLoad);
      this.Controls.Add(this.textBoxLoad);
      this.Controls.Add(this.buttonHeatMesh);
      this.Controls.Add(this.recIdLabel);
      this.Controls.Add(this.nextRecordingButton);
      this.Controls.Add(this.groupBox4);
      this.Controls.Add(this.recDurationLabel);
      this.Controls.Add(this.annotationBox);
      this.Controls.Add(this.groupBox3);
      this.Controls.Add(this.groupBox2);
      this.Controls.Add(this.depthFramesCheckbox);
      this.Controls.Add(this.showColorFramesCheckbox);
      this.Controls.Add(this.sliderPicBar);
      this.Controls.Add(this.timeScrollerB);
      this.Controls.Add(this.label4);
      this.Controls.Add(this.jointIndexUpDown);
      this.Controls.Add(this.cameraUpDown);
      this.Controls.Add(this.CameraLabel);
      this.Controls.Add(this.timeScrollerA);
      this.Controls.Add(this.groupBox1);
      this.Font = new System.Drawing.Font("Calibri", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
      this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedDialog;
      this.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
      this.MaximizeBox = false;
      this.Name = "UIWindow";
      this.Text = "Waiting for connection from application...";
      this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.UIWindow_FormClosing);
      this.Load += new System.EventHandler(this.UIWindow_Load);
      this.KeyUp += new System.Windows.Forms.KeyEventHandler(this.UIWindow_KeyUp);
      this.groupBox1.ResumeLayout(false);
      this.groupBox1.PerformLayout();
      ((System.ComponentModel.ISupportInitialize)(this.minSegDiagUpDown)).EndInit();
      ((System.ComponentModel.ISupportInitialize)(this.selectSegIdUpDown)).EndInit();
      ((System.ComponentModel.ISupportInitialize)(this.segColorWeightUpDown)).EndInit();
      ((System.ComponentModel.ISupportInitialize)(this.minVertsUpDown)).EndInit();
      ((System.ComponentModel.ISupportInitialize)(this.segKthreshUpDown)).EndInit();
      ((System.ComponentModel.ISupportInitialize)(this.cameraUpDown)).EndInit();
      ((System.ComponentModel.ISupportInitialize)(this.integrationWindowUpDown)).EndInit();
      ((System.ComponentModel.ISupportInitialize)(this.jointIndexUpDown)).EndInit();
      ((System.ComponentModel.ISupportInitialize)(this.activationRadiusUpDown)).EndInit();
      ((System.ComponentModel.ISupportInitialize)(this.sliderPicBar)).EndInit();
      this.groupBox2.ResumeLayout(false);
      this.groupBox2.PerformLayout();
      this.groupBox3.ResumeLayout(false);
      this.groupBox3.PerformLayout();
      this.annotationBox.ResumeLayout(false);
      this.annotationBox.PerformLayout();
      this.groupBox4.ResumeLayout(false);
      this.groupBox4.PerformLayout();
      this.ModelsGroup.ResumeLayout(false);
      this.ModelsGroup.PerformLayout();
      this.visualizationGroupBox.ResumeLayout(false);
      this.visualizationGroupBox.PerformLayout();
      this.modeGroupbox.ResumeLayout(false);
      this.modeGroupbox.PerformLayout();
      this.ResumeLayout(false);
      this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Timer timerProcessMessages;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.Timer timerInitialize;
        private System.Windows.Forms.Timer timerDisconnectCheck;
        private System.Windows.Forms.HScrollBar timeScrollerA;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label CameraLabel;
        private System.Windows.Forms.NumericUpDown cameraUpDown;
        private System.Windows.Forms.NumericUpDown integrationWindowUpDown;
        private System.Windows.Forms.NumericUpDown jointIndexUpDown;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label actRlabel;
        private System.Windows.Forms.NumericUpDown activationRadiusUpDown;
        private System.Windows.Forms.CheckBox activationMapCheckbox;
        private System.Windows.Forms.CheckBox showSkeletonCheckbox;
        private System.Windows.Forms.CheckBox hideUnactivatedCheckbox;
        private System.Windows.Forms.CheckBox showGazeCheckbox;
        private System.Windows.Forms.CheckBox showSegmentationCheckbox;
        private System.Windows.Forms.NumericUpDown minVertsUpDown;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.NumericUpDown segKthreshUpDown;
        private System.Windows.Forms.Label kThreshLabel;
        private System.Windows.Forms.CheckBox showOBBsCheckBox;
        private System.Windows.Forms.CheckBox showScanCheckBox;
        private System.Windows.Forms.CheckBox showBodyPointCheckBox;
        private System.Windows.Forms.Button segSaveButton;
        private System.Windows.Forms.NumericUpDown segColorWeightUpDown;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.NumericUpDown minSegDiagUpDown;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.NumericUpDown selectSegIdUpDown;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.CheckBox showInteractionButton;
        private System.Windows.Forms.Button jointSegKnnButton;
        private System.Windows.Forms.HScrollBar timeScrollerB;
        private System.Windows.Forms.PictureBox sliderPicBar;
        private System.Windows.Forms.CheckBox showColorFramesCheckbox;
        private System.Windows.Forms.CheckBox depthFramesCheckbox;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.GroupBox annotationBox;
        private System.Windows.Forms.TextBox annotationTextbox;
        private System.Windows.Forms.Button annotationPushButton;
        private System.Windows.Forms.Button annotationDumpButton;
        private System.Windows.Forms.CheckBox playRangeButton;
        private System.Windows.Forms.ListBox annotationsBox;
        private System.Windows.Forms.Button annDeleteButton;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.Button createDBbutton;
        private System.Windows.Forms.Button nextPoseButton;
        private System.Windows.Forms.CheckBox showInteractionHeatmap;
        private System.Windows.Forms.CheckBox showSegVerbProbButton;
        private System.Windows.Forms.Button recomputeHeatmapButton;
        private System.Windows.Forms.CheckBox interactionHeatmapUseMax;
        private System.Windows.Forms.Button segKnnButton;
        private System.Windows.Forms.CheckBox jointSegKnnUseCurr;
        private System.Windows.Forms.CheckBox checkBoxShowClusterAssignment;
        private System.Windows.Forms.CheckBox interactionHeatmapShowAngle;
        private System.Windows.Forms.Button nextRecordingButton;
        private System.Windows.Forms.Label recIdLabel;
        private System.Windows.Forms.CheckBox constrainZupCheckbox;
        private System.Windows.Forms.CheckBox useSingleSkelForHeatmapCheckbox;
        private System.Windows.Forms.Button loadAnnotationButton;
        private System.Windows.Forms.Button trainAllButton;
        private System.Windows.Forms.Button trainButton;
        private System.Windows.Forms.Button testButton;
        private System.Windows.Forms.Button cvButton;
        private System.Windows.Forms.Button buttonHeatMesh;
        private System.Windows.Forms.TextBox textBoxLoad;
        private System.Windows.Forms.Button buttonLoad;
        private System.Windows.Forms.CheckBox dumpImagesCheckbox;
        private System.Windows.Forms.CheckBox paintAllJointsCheckbox;
        private System.Windows.Forms.Button helpButton;
        private System.Windows.Forms.Button segAnnotateButton;
        private System.Windows.Forms.Button segLoadButton;
        private System.Windows.Forms.CheckBox showSegGroupOBBsCheckBox;
        private System.Windows.Forms.CheckBox showKNNIGSimCheckbox;
        private System.Windows.Forms.CheckBox showPIGSimCheckbox;
        private System.Windows.Forms.CheckBox showInteractionFrameCheckbox;
        private System.Windows.Forms.GroupBox ModelsGroup;
        private System.Windows.Forms.Button loadModelButton;
        private System.Windows.Forms.CheckBox showModelsCheckbox;
        private System.Windows.Forms.TextBox modelTextBoxLoad;
        private System.Windows.Forms.CheckBox showModelSegmentationCheckbox;
        private System.Windows.Forms.CheckBox loadModelSegFromFileCheckBox;
        private System.Windows.Forms.CheckBox showModelVoxelsCheckBox;
        private System.Windows.Forms.Button segPerJointClassifyButton;
        private System.Windows.Forms.Label recDurationLabel;
        private System.Windows.Forms.GroupBox visualizationGroupBox;
        private System.Windows.Forms.GroupBox modeGroupbox;
        private System.Windows.Forms.RadioButton poseModeRadio;
        private System.Windows.Forms.RadioButton replayModeRadio;
        private System.Windows.Forms.ComboBox interactionDropdown;
        private System.Windows.Forms.ComboBox interactionTypeDropdown;
        private System.Windows.Forms.ComboBox classifierTypeDropdown;
        private System.Windows.Forms.ComboBox modelLoadTypeDropdown;
        private System.Windows.Forms.Button testInteractionButton;
        private System.Windows.Forms.Button nextISetButton;
        private System.Windows.Forms.CheckBox showScanLabeledVoxelsCheckBox;
        private System.Windows.Forms.CheckBox showScanVoxelsCheckBox;
        private System.Windows.Forms.CheckBox showAllInteractionSetsCheckBox;
        private System.Windows.Forms.CheckBox showObjOBBsCheckBox;
        private System.Windows.Forms.CheckBox showModelInteractionMapCheckbox;
    }
}

