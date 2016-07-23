namespace UIWindow
{
    partial class AnnotatorWindow
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
      this.SaveButton = new System.Windows.Forms.Button();
      this.annotationBoxLabel = new System.Windows.Forms.Label();
      this.clearButton = new System.Windows.Forms.Button();
      this.annotationsTable = new System.Windows.Forms.DataGridView();
      this.LoadButton = new System.Windows.Forms.Button();
      this.Id = new System.Windows.Forms.DataGridViewTextBoxColumn();
      this.Label = new System.Windows.Forms.DataGridViewTextBoxColumn();
      this.Info = new System.Windows.Forms.DataGridViewTextBoxColumn();
      this.objectId = new System.Windows.Forms.DataGridViewTextBoxColumn();
      ((System.ComponentModel.ISupportInitialize)(this.annotationsTable)).BeginInit();
      this.SuspendLayout();
      // 
      // SaveButton
      // 
      this.SaveButton.Font = new System.Drawing.Font("Calibri", 12F, System.Drawing.FontStyle.Bold);
      this.SaveButton.Location = new System.Drawing.Point(471, 52);
      this.SaveButton.Margin = new System.Windows.Forms.Padding(4);
      this.SaveButton.Name = "SaveButton";
      this.SaveButton.Size = new System.Drawing.Size(160, 39);
      this.SaveButton.TabIndex = 2;
      this.SaveButton.Text = "Save";
      this.SaveButton.UseVisualStyleBackColor = true;
      this.SaveButton.Click += new System.EventHandler(this.SaveButton_Click);
      // 
      // annotationBoxLabel
      // 
      this.annotationBoxLabel.AutoSize = true;
      this.annotationBoxLabel.Location = new System.Drawing.Point(13, 18);
      this.annotationBoxLabel.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
      this.annotationBoxLabel.Name = "annotationBoxLabel";
      this.annotationBoxLabel.Size = new System.Drawing.Size(93, 19);
      this.annotationBoxLabel.TabIndex = 4;
      this.annotationBoxLabel.Text = "Annotations";
      // 
      // clearButton
      // 
      this.clearButton.Location = new System.Drawing.Point(471, 318);
      this.clearButton.Name = "clearButton";
      this.clearButton.Size = new System.Drawing.Size(160, 37);
      this.clearButton.TabIndex = 5;
      this.clearButton.Text = "Clear";
      this.clearButton.UseVisualStyleBackColor = true;
      this.clearButton.Click += new System.EventHandler(this.clearButton_Click);
      // 
      // annotationsTable
      // 
      this.annotationsTable.AllowUserToAddRows = false;
      this.annotationsTable.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize;
      this.annotationsTable.Columns.AddRange(new System.Windows.Forms.DataGridViewColumn[] {
            this.Id,
            this.Label,
            this.Info,
            this.objectId});
      this.annotationsTable.Location = new System.Drawing.Point(17, 52);
      this.annotationsTable.MultiSelect = false;
      this.annotationsTable.Name = "annotationsTable";
      this.annotationsTable.Size = new System.Drawing.Size(447, 303);
      this.annotationsTable.TabIndex = 6;
      this.annotationsTable.CellValueChanged += new System.Windows.Forms.DataGridViewCellEventHandler(this.annotationsTable_CellValueChanged);
      this.annotationsTable.CurrentCellChanged += new System.EventHandler(this.annotationsTable_CurrentCellChanged);
      this.annotationsTable.SelectionChanged += new System.EventHandler(this.annotationsTable_SelectionChanged);
      this.annotationsTable.UserDeletingRow += new System.Windows.Forms.DataGridViewRowCancelEventHandler(this.annotationsTable_UserDeletingRow);
      // 
      // LoadButton
      // 
      this.LoadButton.Location = new System.Drawing.Point(472, 98);
      this.LoadButton.Name = "LoadButton";
      this.LoadButton.Size = new System.Drawing.Size(160, 39);
      this.LoadButton.TabIndex = 7;
      this.LoadButton.Text = "Load";
      this.LoadButton.UseVisualStyleBackColor = true;
      this.LoadButton.Click += new System.EventHandler(this.LoadButton_Click);
      // 
      // Id
      // 
      this.Id.HeaderText = "Id";
      this.Id.MaxInputLength = 10;
      this.Id.Name = "Id";
      this.Id.ReadOnly = true;
      // 
      // Label
      // 
      this.Label.HeaderText = "Label";
      this.Label.Name = "Label";
      // 
      // Info
      // 
      this.Info.HeaderText = "Info";
      this.Info.Name = "Info";
      this.Info.ReadOnly = true;
      // 
      // objectId
      // 
      this.objectId.HeaderText = "Object Id";
      this.objectId.Name = "objectId";
      // 
      // AnnotatorWindow
      // 
      this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 19F);
      this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
      this.ClientSize = new System.Drawing.Size(644, 380);
      this.Controls.Add(this.LoadButton);
      this.Controls.Add(this.annotationsTable);
      this.Controls.Add(this.clearButton);
      this.Controls.Add(this.annotationBoxLabel);
      this.Controls.Add(this.SaveButton);
      this.Font = new System.Drawing.Font("Calibri", 12F, System.Drawing.FontStyle.Bold);
      this.Margin = new System.Windows.Forms.Padding(4);
      this.Name = "AnnotatorWindow";
      this.Text = "Annotations";
      this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.AnnotatorWindow_FormClosing);
      this.Load += new System.EventHandler(this.AnnotatorWindow_Load);
      ((System.ComponentModel.ISupportInitialize)(this.annotationsTable)).EndInit();
      this.ResumeLayout(false);
      this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button SaveButton;
        private System.Windows.Forms.Label annotationBoxLabel;
        private System.Windows.Forms.Button clearButton;
        private System.Windows.Forms.DataGridView annotationsTable;
        private System.Windows.Forms.Button LoadButton;
        private System.Windows.Forms.DataGridViewTextBoxColumn Id;
        private System.Windows.Forms.DataGridViewTextBoxColumn Label;
        private System.Windows.Forms.DataGridViewTextBoxColumn Info;
        private System.Windows.Forms.DataGridViewTextBoxColumn objectId;
    }
}
