# TurtleBot4-Guide
```latex ... ```
\documentclass[12pt,a4paper,openany]{report}
\usepackage[
    a4paper,
    top=2cm,bottom=2cm,
    outer=2cm,inner=3cm,
    includeheadfoot
]{geometry} 
\usepackage{hyperref}
\hypersetup{
    colorlinks=true,    % Enables colored links
    linkcolor=black,    % Color of internal links (sections, pages, etc.)
    urlcolor=blue,      % Color of external links (URLs)
    citecolor=black     % Color of citation links
    % You can adjust colors as needed or set them all to black to remove coloring
}
\usepackage{graphicx}
\usepackage{icomma}
\usepackage{minted}
\usepackage[a-1a]{pdfx}
\usepackage[output-decimal-marker={,}]{siunitx}
\usepackage{subcaption}
\usepackage{svg}
\usepackage[english]{babel}
\selectlanguage{english} % Required for inserting images
\usepackage{mathptmx}
\usepackage{times}
\usepackage{amsmath}
\usepackage{amssymb}
\usepackage[ruled,vlined,algochapter]{algorithm2e}
\usepackage{booktabs}
\usepackage{longtable}
\usepackage{multirow}
\usepackage{multicol}
\usepackage{etex}
\usepackage{hyperref}
\usepackage{lineno}
\usepackage{epstopdf}
\usepackage{colortbl}
\usepackage{subcaption}
\usepackage{picture}
\usepackage{lipsum}
\usepackage{tabularx}
\usepackage{amssymb}
\usepackage{rotating}

\title{Advanced encrypted command laws}
\author{Ahmad Ramadan}
\date{September 2024}
\usepackage{titlesec}
\titleformat{\chapter}[block]
{\normalfont\Huge\bfseries}{Chapter \thechapter:}{0.3cm}{\Huge}
\titlespacing*{\chapter}{0pt}{-20pt}{15pt}
\usepackage{fancyhdr}
% Configure the page header to display chapter number and name
\pagestyle{fancy}
\fancyhf{} % clear existing header/footer entries
\fancyhead[R]{} % Clear any right header content
\fancyfoot[R]{\thepage} % Place page number in the right footer
\renewcommand{\headrulewidth}{0pt} % Remove header line

% Redefine the "plain" page style to match the custom footer
\fancypagestyle{plain}{%
  \fancyhf{} % clear existing header/footer entries
  \fancyfoot[R]{\thepage} % Place page number in the right footer
  \renewcommand{\headrulewidth}{0pt} % Remove header line
}
\setlength{\parskip}{10pt plus 1pt minus 1pt}
\setlength{\parindent}{0pt}
\makeatletter
\renewcommand{\@seccntformat}[1]{\csname the#1\endcsname\hspace{8pt}}
\makeatother

\begin{document}


\input{frontpage}

\tableofcontents

\chapter*{Acknowledgments}
I would like to express my sincere gratitude and appreciation to Prof. Dr. Timothée Schmoderer, for his invaluable guidance and constant supervision, as well as for providing necessary information regarding the project. His expertise and encouragement have been instrumental in shaping the direction and outcome of this work.

I am also grateful to the head of the Electrical and Electonics Department Dr. Youssef Harkouss, for her tireless advocacy on behalf of the department’s students throughout our years of education.

Moreover, I would like to acknowledge Dr. Mazen Ghandour, director of the Faculty of Engineering – Branch III, for his responsive support during moments of challenge. His willingness to address concerns and provide solutions played a significant role in overcoming obstacles during these last years.

Finally, I would like to express my sincere appreciation to the dedicated members of the jury, for their instrumental role in the final evaluation of this project.
\begin{flushright}
Sincerely,\\
Ahmad Ramadan
\end{flushright}

\addcontentsline{toc}{section}{Acknowledgments}

\newpage

\chapter*{Abstract}
In the realm of modern control systems, particularly with the emergence of cloud computing, ensuring the security of cyber-physical systems has become paramount. This internship delves into the realm of advanced encrypted command laws as a solution to mitigate security risks. By employing homomorphic encryption algorithms, which allow operations to be performed directly on encrypted data, the aim is to extend the applicability of encryption methods to control systems while ensuring robustness. The internship, conducted at the Prisme laboratory, focuses on implementing and evaluating the performance of these algorithms in controlling dynamical systems, particularly nonlinear ones. Key objectives include exploring new algorithms, studying the stability, convergence, and robustness properties of encrypted control laws, and assessing their performance metrics such as tracking quality, computation time, and resilience to uncertainties. The internship also involves the application of linearization techniques to extend the results obtained to non-linear systems, exemplified by the case of a mobile robot. This work contributes to advancing the understanding and application of encryption in control systems, addressing critical security challenges in modern cyber-physical systems.\\

 \textbf{Keywords}: Homomorphic Encryption, LWE (Learning With Errors), CKKS Encryption Scheme, TurtleBot4, Nonlinear Control Systems, Cybersecurity, Robust Control with Encryption, Cryptography.
\addcontentsline{toc}{section}{Abstract}

\listoffigures
\addcontentsline{toc}{section}{List of Figures}

\listoftables
\addcontentsline{toc}{section}{List of Tables}

\newpage

\begin{flushleft}
    \huge\textbf{List of Abbreviations}
\end{flushleft}
\begin{tabular}{c@{\hspace{1.5cm}}p{12cm}}
\textbf{Abbreviation} & \textbf{Definition} \\
FHE & Fully Homomorphic Encryption \\
LWE & Learning with Errors Encryption\\
RLWE & Learning with Errors Encryption over Rings \\
CKKS & Cheon-Kim-Kim-Song Fully Homomorphic Encryption Scheme \\
ROS & Robot Operating System \\

\end{tabular}
\addcontentsline{toc}{section}{List of Abbreviations}

\newpage

\begin{flushleft}
    \huge\textbf{Nomenclature}
\end{flushleft}
\begin{tabular}{c@{\hspace{1.5cm}}p{12cm}}
$\mathbb{R}$ & Set of Real Numbers \\
$\mathbb{Z}$ & Set of Integers \\
$\mathbb{N}$ & Set of Natural Numbers \\
$\mathbb{R}^+$ & Set of Non-Negative Real Numbers \\
$\mathbb{R}^{++}$ & Set of Positive Real Numbers \\
$\mathbb{Z}^{++}$ & Set of Positive Integers \\
\( C \) & The Ciphertext Space\\
$\mathbb{Z}_q$ &  Finite Field of Integers Modulo q \\
$\mathbb{Z}_{nq}$ &  Set of Integers Modulo q with Dimension n \\
$R_q$ & Polynomials with Integer Coefficients Bounded by q \\

\end{tabular}
\addcontentsline{toc}{section}{Nomenclature}

\newpage

\chapter*{General Introduction}
In today's digital age, keeping our interconnected systems secure is crucial. As more data processing moves to remote servers in the cloud, protecting these systems from cyber attacks becomes increasingly important.Attacks such as eavesdropping and remote control pose significant risks, underscoring the need for robust cybersecurity measures.

Homomorphic encryption offers a promising solution by enabling operations on encrypted data, preserving confidentiality while allowing for meaningful computation. However, applying this technique to control systems presents unique challenges, particularly in ensuring stability, precision, and security.

This study explores the implementation of advanced encrypted command laws using state-of-the-art encryption schemes, aiming to bolster the resilience of cyber-physical systems against evolving threats.

Chapter I presents the foundational tools employed in this study, including the groundbreaking encryption schemes Learning With Errors (LWE) and the Cheon-Kim-Kim-Song (CKKS) method, along with the implementation of Robot Operating System 2 (ROS 2) on the TurtleBot robot platform.

Chapter II delves into the objectives of the research, outlining the model utilized for analysis. Furthermore, this chapter presents the theoretical underpinnings of the study and discusses the anticipated results based on theoretical frameworks.

Chapter III presents the numerical simulations and real-world experiments conducted on the TurtleBot platform. These experiments offer practical insights into the performance and effectiveness of the proposed methodology, validating theoretical findings through empirical observation.
\addcontentsline{toc}{section}{General Introduction}

\newpage

\chapter{Foundational Framework: Advanced Encryption Schemes and Robotic Platform Integration}

\section{Introduction}
In the realm of cybersecurity and robotic systems, the integration of advanced encryption schemes represents a pivotal frontier in safeguarding data integrity and system functionality. 

At the heart of our investigation lies the convergence of advanced encryption schemes, characterized by their ability to ensure data confidentiality and computational privacy in the face of constantly changing threats. Through a systematic analysis of cutting-edge cryptographic techniques such as homomorphic encryption, differential privacy, and secure multiparty computation, we demonsrate the theoretical foundations and practical consequences of employing these methodologies within the context of robotic systems.

Furthermore, delving into the complexities of robotic platform integration entails examining the challenges and opportunities associated with integrating encrypted command laws with various hardware architectures and communication protocols. From establishing secure communication channels to creating encryption-aware control algorithms, the different aspects of incorporating advanced encryption schemes into the fabric of robotic platforms are investigated.

\section{Encryption Schemes}

\subsection{General Background}
Homomorphic encryption is a form of encryption that allows computations to be performed on encrypted data without first having to decrypt it. The resulting computations are left in an encrypted form which, when decrypted, result in an output that is identical to that produced had the operations been performed on the unencrypted data. Homomorphic encryption can be used for privacy-preserving outsourced storage and computation. This allows data to be encrypted and out-sourced, all while encrypted.

Fully Homomorphic Encryption is a secure encryption method that enables computations to be executed on encrypted data without decrypting it. In simple terms, FHE allows for secure computations on encrypted data without exposing the original text (plaintext). This makes FHE a powerful tool for protecting sensitive data by allowing secure computation without the risk of data leaks, unauthorized access, or unwanted authorization.

FHE, has the profound potential to greatly enhance our ability to handle sensitive data securely. This cutting-edge technology enables secure computation in scenarios that cannot be achieved using traditional encryption techniques.

\subsection{Learning with Errors Encryption over Rings}
In cryptography, Learning with Errors encryption presents a fundamental mathematical problem utilized to construct robust encryption algorithms. The core concept revolves around representing confidential data through equations interwoven with errors. In essence, LWE offers a mechanism to obscure sensitive information by injecting noise into its representation.

Specifically, "RLWE encryption" refers to lattice-based cryptography, prominently featured in encryption methodologies such as the Ring Learning With Errors encryption scheme. RLWE encapsulates a computational challenge wherein the primary objective is to decipher a secret, random vector "s" from its inner product entangled with a set of noisy samples.

In this section, we begin to describe a homomorphic public-key encryption scheme based on RLWE. In order to guarantee correctness and security, we set parameters below which depend on the security parameter \(\kappa\). Now we define the ring \( R = \mathbb{Z}[X] / f(x) \) with the cyclotomic polynomial \( f(x) = x^{n} + 1 \). We set the error distribution \( \chi \) to be the truncated discrete Gaussian \( \text{D}_{\mathbb{Z}^n,r} \) with a specified standard deviation.

\subsubsection{Cryptosystem Mechanics and Homomorphic Properties}
This subsection introduces the cryptosystem utilized in this paper. For an unencrypted value, we call it a plaintext, while an encrypted value is called a ciphertext. Let \(C\) denote the ciphertext space. The encryption map, denoted by \(\text{Enc}: \mathbb{Z}_q \rightarrow C\), represents the process of encrypting a plaintext, and the decryption map, denoted by \(\text{Dec}: C \rightarrow \mathbb{Z}_q\), represents the process of decrypting a ciphertext. We will now provide a brief description of LWE-based encryption as follows:

\begin{itemize}
    \item Choose a secret key \(s_k \in \mathbb{Z}_{nq}\), a random vector \(a \in \mathbb{Z}_{nq}\), and a small random error \(e \in \mathbb{Z}_q\).
    
    \item \textbf{Encryption:} \\
    For a plaintext \(m \in \mathbb{Z}_q\), the corresponding ciphertext is computed as
    \begin{equation}
    \text{Enc}(m) = \begin{pmatrix} (m + s_k^T \cdot a + e ) \\ a \end{pmatrix} \bmod q = c \in \mathbb{Z}_{n+1,q}.
    \end{equation}
    
    To encrypt a message \(m\), a fresh LWE tuple \((b, a^T)\) is generated, and the encryption is performed via
    \begin{equation}
    \ c = \text{Enc}_{{s_k}}(m) = (b + m, a^T) \mod q = (b, a^T).
    \end{equation}
    Where 
   \begin{equation}
    b = a^{T} s_k + e + m \mod q
    \end{equation}
    where $a^{T}$ represents the transposed vector of $a$, $s_k$ is the secret key, $e$ denotes the error term, $m$ is the plaintext, and $q$ is the modulus.

    \item \textbf{Decryption:} \\
    For the ciphertext \(c \in C\) corresponding to the plaintext \(m \in \mathbb{Z}_q\), define
    \begin{equation}
    \text{Dec}(c) =\begin{pmatrix} (1 - s_k^T) \end{pmatrix} c \bmod q.
    \end{equation}
    
    Where the decryption is the inner product of c and $(s_k, 1)$ leading to
    \begin{equation}
    \text{Dec}(c) = b - a^T s_k \mod q = z + e \mod q.
    \end{equation}
    
    It is obvious that \(\text{Dec}(c) = m+e\). To deal with the error \(e\), let us consider the encryption with gain \(G > 0\) as
    \begin{equation}
    c_G = \text{Enc}(Gm) = \left(Gm + s_k \cdot a + e \cdot a\right) \bmod q.
    \end{equation}
    
    Then, with \(G \in \mathbb{Z}_q\) being a positive gain such that \(G > 2e\), the plaintext can be recovered as
    \begin{equation}
    \ \lfloor \frac{{Dec}(c_G)}{G} \rceil = \lfloor  m + \frac{e}{G} \rceil = m.
    \end{equation}
    
    Hereafter, when considering a ciphertext, we always assume that it is encrypted with a suitable gain \(G\).
    
    \item[] \hspace{1em} \textbf{1)} \textbf{Additive property:} 
    \begin{itemize} 
        \item For \(c_1, c_2 \in C\), \(c = c_1 + c_2 \bmod q\), and \(c' = kc_1 \bmod q\) with \(k \in \mathbb{Z}_q\), one has
        \begin{flalign}
        \begin{aligned}
            &\text{Dec}(c) = \text{Dec}(c_1) + \text{Dec}(c_2) & \\
            &\text{Dec}(c') = k \cdot \text{Dec}(c_1) &
        \end{aligned}
        \end{flalign}
    \end{itemize}
    
     \item[] \hspace{1em} \textbf{2)} \textbf{Multiplication:} 
     \begin{itemize} 
        \item To have the ability of multiplication, let us introduce a separate algorithm for encrypting the multipliers utilized in [31]. Consider the ciphertexts \(c_1, c_2 \in \mathbb{Z}_{n+1,q}\) corresponding to the plaintexts \(m_1, m_2 \in \mathbb{Z}_q\). Choose \(q \in \mathbb{Z}^{+}\) such that there exist \(l, d \in \mathbb{Z}_q\) and \(ld = q\). Let \(D(\cdot)\) denote the function that decomposes.
        Then, the multiplication between two ciphertexts \(c_1\) and \(c_2\) is defined as
        
        \begin{equation}
        \ c_1 \otimes c_2 = \text{Enc}'(m_1) \cdot D(c_2) \mod q;
        \end{equation}
        
        To see the homomorphic property, we note that
        \begin{equation}
        \text{Dec}(\text{Enc}'(m_1) \cdot D(c_2)) = m_1 \cdot m_2 + e \mod q; 
        \end{equation}
        
        with \( e = [e_1, \dots, e_{d(n+1)}] D(c_2) \).

    \end{itemize}
\end{itemize}

\newpage

\subsection{Cheon-Kim-Kim-Song Encryption}
The Cheon-Kim-Kim-Song fully homomorphic encryption scheme allows to perform homomorphic computations over real numbers. This procedure supports
approximate arithmetic over complex numbers (hence, real numbers) with a level of error comparable to the one made by floating point arithmetic on traditional computers. The CKKS framework consists of five main algorithms: encryption,
decryption, addition, multiplication, and rescaling.\\
The first two algorithms transform plaintexts (complex vectors) into ciphertexts (polynomial elements) and vice-versa; the next two perform homomorphic operations on encrypted data; the rescaling procedure is used to maintain the size of the message and to control the growth of the error after homomorphic operations. Nevertheless, in the sequel, we focus on the intuition behind the encryption/decryption procedure and the evaluation of homomorphic operations.

\begin{itemize}
    \item \textbf{CKKS parameters:} \\
    The CKKS scheme uses the following parameters: $N$ is a power of two giving the dimension of the ciphertext domain; $q_L = \Delta_L q_0$ is the modulus of the space, where $\Delta$ is the scaling factor, $L$ is an integer chosen according to the depth of the evaluation circuit, and $q_0$ is the basis modulus. Moreover, the polynomial ring space $R_q$ is defined by $\mathbb{Z}_q[X]/(X^N + 1)$. Elements of $R_q$ are polynomials with integer coefficients bounded by $q$.

    \item \textbf{Encryption/Decryption:} \\
    In the CKKS encryption procedure, the first step is an encoding step: the clear text message $m \in \mathbb{C}^{N/2}$ is transformed into an element of the polynomial ring $R_q$. This operation is completely transparent, thus, without loss of generality, we assume that the plaintexts are already polynomials. The encryption/decryption procedures rely on the generation of a public key and a secret key. The secret key $s_k$ is used for the decryption (it can also be used for encryption in a symmetric encryption scheme framework) and is taken as a random polynomial of degree $N$ with coefficients in $\{-1, 0, 1\}$. The public key is a pair of polynomials $(PK1, PK2)$ given by:
    \begin{align}
    \begin{aligned}
        PK_1 &= -a \cdot s_k + e \mod q_L \\
        PK_2 &= a
    \end{aligned}
    \end{align}
    
    where $a$ is a random polynomial sampled uniformly from $R_q^L$ and $e$ is a random error polynomial sampled from the discrete Gaussian distribution with variance $\sigma^2$. To encrypt a plaintext $m$, we generate three small random polynomials $\mu$ with coefficients in $\{-1, 0, 1\}$, $e1$ and $e2$ from the discrete Gaussian distribution with variance $\sigma^2$, and we construct the ciphertext $\text{Enc}(m) = c \in R_{2q}^L$ given by the pair $(c_1, c_2)$ defined as follows:
    
    \begin{align}
        \begin{aligned}
            c_1 &= PK_1 \cdot \mu + e_1 + \left\lfloor \Delta m \right\rfloor \mod q_L \\
            c_2 &= PK_2 \cdot \mu + e_2 \mod q_L
        \end{aligned}
    \end{align}

    The encryption process injects an error in the message. Hence, to preserve the precision of the message, we multiply it by a scaling factor $\Delta > 1$. The decryption is performed by evaluating the ciphertext on the secret key to generate an approximate value $m'$ of the plaintext $m$:
    \begin{equation}
    m' = \text{Dec}(c) = \frac{1}{\Delta}(c_1 + c_2 \cdot s_k \mod q_L)
    \end{equation}
    We say that a ciphertext $c$ contains a noise $e$ if the decrypted value $m'$ differs from the exact plaintext $m$ by $e$.

    \item \textbf{Homomorphic operations:} \\
    The main advantage of using a FHE scheme is to perform arithmetic operations in the cipher domain that are equivalent to the operations in the clear domain. With CKKS, the addition is straightforward: we simply add the corresponding polynomials of each ciphertext. Consider two ciphertexts 
    $c^1 = (c^1_1, c^1_2)$ and $c^2 = (c^2_1, c^2_2)$, the addition is defined as:
    \begin{equation}
    c^1 \oplus c^2 = \left( c^1_1 + c^2_1 \mod q_L, c^1_2 + c^2_2 \mod q_L \right)
    \end{equation}
    The multiplication consists of two steps: the multiplication of the polynomials and a rescaling step which aims at reducing the noise introduced by the first step. We set $(d_0, d_1, d_2) = (c^1_1 c^2_1, c^1_2 c^2_1 + c^2_1 c^1_2, c^1_2 c^2_2)$. The multiplication of two ciphertexts is:
    \begin{equation}
    c^1 \otimes c^2 = \lfloor \frac{1}{\Delta} \left( (d_0, d_1) + \lfloor P^{-1} d_2 E_k \rceil \mod q_L) \right \rceil
    \end{equation}
    where $E_k = (-a' \cdot s_k + e' + P \cdot s_k^2 \mod P{q_L}, a')$ is the evaluation key given by an integer $P$, and two polynomials $a'$ and $e'$ constructed like for the public key. 
\end{itemize}

\section{Implementation of TurtleBot with ROS2: Development and Integration}

\subsection{Background}
ROS 2 stands for "Robot Operating System 2". It's an open-source framework for writing robot software. It's designed to be a flexible platform for developing robotics applications. ROS 2 is a successor to ROS, aiming to address limitations and improve upon its predecessor's functionalities. It offers improved real-time capabilities, better support for various hardware platforms, and enhanced security features, among other advancements. \\
ROS 2 relies on the notion of combining workspaces using the shell environment. “Workspace” is a ROS term for the location on your system where you’re developing with ROS 2. The core ROS 2 workspace is called the underlay. Subsequent local workspaces are called overlays. When developing with ROS 2, you will typically have several workspaces active concurrently. \\
Combining workspaces makes developing against different versions of ROS 2, or against different sets of packages, easier. It also allows the installation of several ROS 2 distributions (distros) on the same computer and switching between them. \\
This is accomplished by sourcing setup files every time you open a new shell, or by adding the source command to your shell startup script once. Without sourcing the setup files, you won’t be able to access ROS 2 commands, or find or use ROS 2 packages. In other words, you won’t be able to use ROS 2.

\subsubsection{TurtleBot4}
The TurtleBot 4 is a ROS 2-based mobile robot intended for education and research. The TurtleBot 4 is capable of mapping the robot's surroundings, navigating autonomously, running AI models on its camera, and more. \\
It uses a Create® 3 as the base platform, and builds on it with the TurtleBot 4 shell and User Interface (UI) board. Inside the shell sits a Raspberry Pi 4B which runs the TurtleBot 4 software. \\
The TurtleBot 4 Lite is a barebones version of the TurtleBot 4. It has just the necessary components for navigation, mapping, and AI applications. The TurtleBot 4 has the same Raspberry Pi 4B, which sits in the cargo bay of the Create® 3, as well as the same RPLIDAR A1M8. The camera on the TurtleBot 4 Lite is the OAK-D-Lite. Additional sensors and payloads can be attached to the Create® 3 faceplate, or placed inside the cargo bay.

\begin{itemize}

    \item \textbf{Sensors:} \\
    
    \item[] \hspace{1em} \textbf{1)} \textbf{RPLIDAR A1M8} 
    \begin{itemize}
        \begin{minipage}{0.65\textwidth}
            \item The RPLIDAR A1M8 is a 360 degree Laser Range Scanner with a 12m range. It is used to generate a 2D scan of the robot's surroundings.
        \end{minipage}
        \begin{minipage}{0.3\textwidth}
            \includegraphics[width=\linewidth]{Images/rplidar_a1m8.png}
        \end{minipage}
    \end{itemize}


    \item[] \hspace{1em} \textbf{2)} \textbf{OAK-D-Pro} 
    \begin{itemize}
        \begin{minipage}{0.65\textwidth}
            \item The OAK-D-Lite camera from Luxonis uses a 4K IMX214 colour sensor along with a pair of OV7251 stereo sensors to produce high quality colour and depth images. The on-board Myriad X VPU gives the camera the power to run computer vision applications, object tracking, and run AI models.
        \end{minipage}
        \begin{minipage}{0.3\textwidth}
            \includegraphics[width=\linewidth]{Images/oak-d-pro.png}
        \end{minipage}
    \end{itemize}

    \item[] \hspace{1em} \textbf{3)} \textbf{OAK-D-Lite} 
    \begin{itemize}
        \begin{minipage}{0.65\textwidth}
            \item The OAK-D-Pro offers all of the same features the OAK-D-Lite has, but uses higher resolution OV9282 stereo sensors and adds an IR laser dot projector and an IR illumination LED. This allows the camera to create higher quality depth images, and perform better in low-light environments.
        \end{minipage}
        \begin{minipage}{0.3\textwidth}
            \includegraphics[width=\linewidth]{Images/oak-d-lite.png}
        \end{minipage}
    \end{itemize}

\end{itemize}

In our project, we are currently focusing on the development and integration of the TurtleBot 4. Below, you can find a table highlighting the key differences between the TurtleBot 4 and the TurtleBot 4 Lite. This comparison will help provide insight into the unique features and specifications of each model, aiding in decision-making and understanding their respective capabilities within our project.

\begin{table}
\centering
\caption{Comparison of TurtleBot 4 Lite and TurtleBot 4}
\label{tab:comparison}
\begin{tabular}{@{}lll@{}}
\toprule
  & \begin{tabular}[c]{@{}l@{}} \textbf{TurtleBot 4 Lite}\end{tabular} & \begin{tabular}[c]{@{}l@{}} \textbf{TurtleBot 4}\end{tabular} \\
\hline 
\multicolumn{3}{c}{\textbf{Weight and Size}} \\ \midrule
Dimensions External (LxWxH) & \begin{tabular}[c]{@{}l@{}}7.5 x 13.3 x 13.4 mm \\ (192 x 339 x 341 in)\end{tabular} & \begin{tabular}[c]{@{}l@{}}13.8 x 13.3 x 13.4 mm \\ (351 x 339 x 341 in)\end{tabular} \\
Weight                      & \begin{tabular}[c]{@{}l@{}}7.2 lbs \\ (3.3 kg)\end{tabular}                              & \begin{tabular}[c]{@{}l@{}}8.6 lbs \\ (3.9 kg)\end{tabular}                              \\
Wheels Diameter             & \begin{tabular}[c]{@{}l@{}}0.55 in \\ (14 mm)\end{tabular}                               & \begin{tabular}[c]{@{}l@{}}0.55 in \\ (14 mm)\end{tabular}                               \\
Ground Clearance            & \begin{tabular}[c]{@{}l@{}}0.17 in \\ (4.5 mm)\end{tabular}                              & \begin{tabular}[c]{@{}l@{}}0.17 in \\ (4.5 mm)\end{tabular}                              \\ \midrule
\multicolumn{3}{c}{\textbf{Performance and Speed}}                                                \\ \midrule
Payload Max (Default)       & 9 kg                                           & 15 kg (Custom Configuration)                  \\
Max Speed                   & \begin{tabular}[c]{@{}l@{}}0.46 m/s \\ (Safe Mode)\end{tabular}                          & \begin{tabular}[c]{@{}l@{}}0.31 m/s \\ (Safe Mode)\end{tabular}                          \\
Max Rotational Speed        & 1.90 s/rad                                     & 1.90 s/rad                                    \\ \midrule
\multicolumn{3}{c}{\textbf{System Power and Battery}}                                              \\ \midrule
Chemistry                   & Lithium Ion                                    & Lithium Ion                                   \\
Nominal Voltage             & 14.4 V                                         & 14.4 V                                        \\
Battery Capacity            & 26 Wh                                          & 26 Wh                                         \\
Charge Time                 & 2.5 hrs                                        & 2.5 hrs                                       \\
Operating Time              & 4.0-2.5 hrs (Dependent on Load)               & 4.0-2.5 hrs (Dependent on Load)              \\ \midrule
\multicolumn{3}{c}{\textbf{Sensors}}                                                                \\ \midrule
LIDAR                       & 1A-RPLIDAR                                     & 1A-RPLIDAR                                    \\
Camera                      & PRO-D-OAK                                      & PRO-D-OAK                                     \\
Other Sensors               & LITE-D-OAK                                     & LITE-D-OAK                                    \\ \midrule
\multicolumn{3}{c}{\textbf{Actuators and Computers}}                                                \\ \midrule
Actuators                   & Motors Drive x2                                & Motors Drive x2                               \\
Computers                   & Pi Raspberry (4 GB)                            & Pi Raspberry (4 GB)                           \\ \midrule
\multicolumn{3}{c}{\textbf{Software}}                                                                \\ \midrule
ROS Version                 & 2.0.04                                         & 2.0.04                                        \\
Operating System            & Ubuntu                                         & Ubuntu                                        \\ \bottomrule
\end{tabular}
\end{table}

\newpage

\begin{itemize}

    \item \textbf{Installing ROS2:} \\
    
    \item[] \hspace{1em} \textbf{1)} \textbf{Implementation of the UTF-8 Format} 
    \begin{verbatim}
    > sudo apt update && sudo apt install locales
    > sudo locale-gen en_US en_US.UTF-8
    > sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    > export LANG=en_US.UTF-8
    \end{verbatim}
    \hspace{0.9cm} We observe the installation with the "locale" command
    \begin{figure}[h]
        \centering
        \includegraphics[width=0.5\textwidth]{Images/Locale Command.jpg}
         \caption{locale Command}
        \label{fig: locale Command}
    \end{figure}

    \item[] \hspace{1em} \textbf{2)} \textbf{Source configuration}
    \begin{verbatim}
    > sudo apt install software-properties-common
    > sudo add-apt-repository universe
    > sudo apt update && sudo apt install curl -y
    > sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key 
    -o /usr/share/keyrings/ros-archive-keyring.gpg
    > echo "deb [arch=$(dpkg --print-architecture) 
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg]
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release 
    && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    \end{verbatim}
        
    \newpage

    \item[] \hspace{1em} \textbf{3)} \textbf{Installing ROS2 packages}
    \begin{itemize}
    \item Apt update
    \end{itemize}
    \begin{verbatim}
    > sudo apt update
    > sudo apt upgrade
    \end{verbatim}

    \begin{itemize}
    \item Installing ROS2 packages 
        \begin{itemize}
        \item To install ROS2, there are 2 choices:
            \begin{itemize}
            \item the full version including graphics libraries, example codes, etc.;
    
            \item the “lite” version with only what is needed to run ROS2
            \end{itemize}
        The second installation is often used for systems where resources are “limited” like a RaspberryPI for example.
        \item Installing ROS2 packages:
\begin{verbatim}
> sudo apt install ros-humble-desktop
\end{verbatim}
        \item Installation of ROS2 Limited:
\begin{verbatim}
> sudo apt install ros-humble-ros-base
\end{verbatim}
        If you go to the official ROS2 Humble installation site, they present a "Development Tool" when installing ROS2. You have the option of installing it for your own reasons. In this document, we don't use it.
        \end{itemize}
    \end{itemize}

    \item[] \hspace{1em} \textbf{4)} \textbf{ROS2 environment configuration}
     \begin{itemize}
     \item To use ROS2, it is necessary to "source" its installation folder in order to use ROS2 commands in terminals. We modify the "bashrc" script as follows.
\begin{verbatim}
> gedit ~/.bashrc
\end{verbatim}
     A text editor window should open. At the end of this text file the following lines and save. The second line enables systems using ROS2 to communicate via WIFI on a network described by the ROS domain ID (default '0').
\begin{verbatim}
> source /opt/ros/humble/setup.bash
> export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
> export ROS_DOMAIN_ID=0
\end{verbatim}
     To update the .bashrc file in your terminal, run the following command.
\begin{verbatim}
> source ~/.bashrc
\end{verbatim}
     \textbf{Note}: You can also restart your terminal
     \end{itemize}

\newpage

    \item[] \hspace{1em} \textbf{5)} \textbf{ROS2 installation test}
     \begin{itemize}
     \item Test nodes are available to verify ROS2 installation. Run two different terminals and run the following commands.

        \begin{itemize}
        \item Terminal 1:
\begin{verbatim}
> ros2 run demo_nodes_cpp talker
\end{verbatim}

        \item Terminal 2:
\begin{verbatim}
> ros2 run demo_nodes_cpp listener
\end{verbatim}
        \end{itemize}
     You need to get similar results (the terminals talk to each other).
    \end{itemize}

    \begin{figure}[h]
        \centering
        \includegraphics[width=0.9\textwidth]{Images/Talker-Listener.jpg}
         \caption{Talker-Listener}
        \label{fig: Talker-Listener}
    \end{figure}

    \item[] \hspace{1em} \textbf{6)} \textbf{Installing the colcon compiler}
     \begin{itemize}
     \item The colcon compiler can be used to build a ROS2 application.\\
     Here are the commands to write in a terminal.
\begin{verbatim}
> sudo apt update
> sudo apt install python3-colcon-common-extensions
\end{verbatim}
     To make the compiler easier to use, we mouse over the compiler path in the .bashrc file as :
\begin{verbatim}
> gedit ~/.bashrc
\end{verbatim}
     Then, in this file, below the ROS2 sourcing in the "ROS2 installation" section, we write the following line and save it. "ROS2 installation" section, we write the following line and save.
\begin{verbatim}
> source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
\end{verbatim}
     To update the .bashrc file in your terminal, run the following command.
\begin{verbatim}
> source ~/.bashrc
\end{verbatim}
     \textbf{Note}: You can also restart your terminal
     \end{itemize}

\end{itemize}

\newpage

\begin{itemize}
    \item \textbf{Turtlebot4 installation and configuration:} \\

    \item[] \hspace{1em} \textbf{1)} \textbf{Preparing the WIFI network} 
    \begin{itemize}
    \item Firstly, it is important to configure a network to support at least two wifi bands (2.4GHz and 5GHz). The minimum hardware needed to monitor a turtlebot4 is:
        \begin{itemize}
        \item A WIFI terminal (or cell phone access point) connected to the Internet
        \item A computer running Linux Ubuntu 22.04 with ROS2 for supervision purposes       
        \item Turtlebot4
        \end{itemize}
    In the case of a network not connected to the Internet, you'll need another computer, preferably running Linux Ubuntu 22.04. The purpose of this computer is to synchronize all equipment connected to the network by broadcasting the date and time using the NTP time protocol.
    \end{itemize}

    \item[] \hspace{1em} \textbf{2)} \textbf{ROS2 package installation for turtlebot4}
    \begin{itemize}
    \item On the supervision computer, open a terminal and enter the following commands:
    \end{itemize}
    \begin{verbatim}
    > sudo apt update && sudo apt install ros-humble-turtlebot4-desktop
    > sudo apt install ros-humble-turtlebot4-description 
    > ros-humble-turtlebot4-msgs 
    > ros-humble-turtlebot4-navigation 
    > ros-humble-turtlebot4-node
    \end{verbatim}

    \item[] \hspace{1em} \textbf{3)} \textbf{Turtlebot4 configuration}
    \begin{itemize}
    \item To configure turtlebot4 correctly, it's important to keep the system up to date with the latest patches. To do this, we first update the turtlebot's components, and then configure their parameters.
        \begin{itemize}
            \item RaspberryPi firmware update \\
            To update the RaspberryPI, you need to extract the micro SD card from the RaspberryPI card. \\
            We now use a micro SD to SD adapter and insert the card into our into the SD drive of our computer running Linux Ubuntu 22.04. \\
            We open the 'disques' or 'disks' utility and select the SD card. We format overwriting existing data with zeros and without partitioning. \\
            We download the latest update from the following site, taking into account the version of ROS2 (here Humble): http://download.ros.org/downloads/turtlebot4/ \\
            We extract the .img file from the downloaded .zip file and enter the following command in a terminal.
\begin{verbatim}
> wget https://raw.githubusercontent.com/turtlebot/turtlebot4_setup/humble/
scripts/sd_flash.sh
> bash sd_flash.sh /chemin/de/l’image.img
\end{verbatim}
            To find the name of the SD card, go to the 'disque' or 'disks' utility select the SD card and find the name next to 'Device' (in our case, we have /dev/mmcblk0'. \\
            When the terminal asks for the SD card name, we enter 'mmcblk0' and continue. \\
            We put the SD card back into turtlebot4 and reassemble the robot.
            
            \item CREATE3 card update \\
            Switch on the turtlebot4 by positioning it on its charging base connected to the mains and wait for the robot to play a sound. \\
            On the supervision computer, download the latest version of Create3 (here Humble H2.6). Then connect to the turtlebot4's wifi network (SSID: 'Turtlebot4' | Mdp: 'Turtlebot4'). \\
            Go to a web browser and enter in the address bar the ip '10.42.0.1:8080'. In the 'Update' tab, follow the update instructions.

            \begin{figure}[h]
            \centering
            \includegraphics[width=0.65\textwidth]{Images/Update Robot.jpg}
            \caption{Update Robot}
            \label{fig: Update Robot}
            \end{figure} 
            
            \item RaspberryPi configuration \\
            Switch on turtlebot4 by positioning it on its charging base connected to the mains and wait for the robot to broadcast a sound. \\
            On the supervision computer, connect to the turtlebot4's wifi network (SSID: 'Turtlebot4' | Mdp: 'Turtlebot4'). Go to the remote access session of the turtlebot4 by typing the following command in a terminal:
\begin{verbatim}
> ssh ubuntu@10.42.0.1
\end{verbatim}
            The session password is 'turtlebot4'. \\
            Once connected to turtlebot4, type the following command to configure the robot's parameters.
\begin{verbatim}
> turtlebot4-setup
\end{verbatim}
            A graphical interface opens. \\
            Go to WIFI-SETUP and connect the robot as a client to your wifi network (it's more optimal to connect the turtlebot4 to 5GHz).
            Save and apply changes (you'll be disconnected from the turtlebot's internal wifi). \\
            Connect to your wifi network and go to the remote access session of the turtlebot4 by typing the following command in a terminal:
\begin{verbatim}
> ssh ubuntu@’ip ecrite sur le turtlebot4’
\end{verbatim}
            If you have a lite version of turtlebot4, type the command '> ros2 topic echo /ip' on your supervision PC to find out the ip of the robot connected to your network. \\
            If you've mistakenly written the SSID and wifi password on the turtlebot4, please visit this site: \\
            https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html . \\
            If you wish to modify the \texttt{ROS\_DOMAIN\_ID} variable, go to the \texttt{ROS-SETUP} menu, then \texttt{BASH-SETUP}. \\ Save and apply the changes.

             \item Create3 card configuration \\
             Go to the turtlebot4 online space with a browser and the following ip: robot ip':8080 and connect the create3 card to your wifi network using the 'connect' tab.
             \begin{figure}[h]
             \centering
             \includegraphics[width=0.65\textwidth]{Images/Connect Robot to WIFI.jpg}
             \caption{Connect Robot to WIFI}
             \label{fig: Connect Robot to WIFI}
             \end{figure} 
             
             Click 'connect' and the robot restarts. \\
              
             \newpage
              
             In the 'Application' tab, then 'Configuration', you can modify the ROS2 parameters (ensure that these parameters are identical to those on the RaspberryPi board). \\

             \begin{figure}[h]
             \centering
             \includegraphics[width=0.65\textwidth]{Images/App Config.jpg}
             \caption{Main Configuration}
             \label{fig: Main Configuration}
             \end{figure} 

             Restart the robot (to switch it off, remove it from the base and press and hold the stop button).
             
        \end{itemize}
    \end{itemize}
           
\newpage

    \item[] \hspace{1em} \textbf{4)} \textbf{NTP protocol configuration (clock synchronization)}
    \begin{itemize}
    \item For the proper operation of the network and the systems installed on it, it's important to synchronize everything to the same time and date. In fact, some community-programmed ROS2 nodes use time-stamped data for their operation their operation. \\
    So we configure the routes and servers to be reached to update the time and date of every date of each device on the wifi network. \\
   
    \item We recommend setting the NTP relay to a fixed IPv4 address. In our case, we manually set these parameters on the ENDORSE PC connected to the network as follows: \\
    IP:192.168.1.32 | Subnet mask: 255.255.255.0 | Default gateway:
    192.168.1.1 \\
        \begin{itemize}
            \item NTP relay configuration under Linux Ubuntu 22.04\\
                \begin{itemize}
                    \item Time zone change 
\begin{verbatim}
>sudo timedatectl set-timezone UTC
\end{verbatim}
    
                    \item Installation of 'ntp' configuration software 
\begin{verbatim}
>sudo apt update && sudo apt install ntp
\end{verbatim}

                    \item Configuring the ntp.conf file 
\begin{verbatim}
>sudo nano /etc/ntp.conf
\end{verbatim}
                    We save the file and restart the service.
\begin{verbatim}
>sudo systemctl restart ntp
\end{verbatim}
                \end{itemize} 

            \item Customer configuration \\
                \begin{itemize}
                    \item Time zone change 
\begin{verbatim}
>sudo timedatectl set-timezone UTC
\end{verbatim}

                    \item Disabling the current ntp manager 
\begin{verbatim}
>sudo timedatectl set-ntp off
\end{verbatim}

                    \item Installation of 'ntp' and 'ntpdate' configuration and synchronization software 
\begin{verbatim}
>sudo apt update && sudo apt install ntp && sudo apt install ntpdate
\end{verbatim}

                    \item Configuring the ntp.conf file 
\begin{verbatim}
>sudo nano /etc/ntp.conf
\end{verbatim}
                \end{itemize}
                                   
        \end{itemize}
     
    \end{itemize}

\newpage

            \begin{itemize} 
                \item For the Create3 client, we go to the Web interface as \texttt{{[}IP\_of\_robot{]}:8080}. \\
                Here we use the following IP '192.168.1.7:8080'.
    
                \begin{figure}[h]
                \centering
                \includegraphics[width=0.65\textwidth]{Images/Create3 Web page.jpg}
                \caption{Create3 Web page}
                \label{fig: Create3 Web page}
                \end{figure}
    
                 \item We enter the 'Edit ntp.conf' menu and enter the following lines (commenting on the default servers).
    
                \begin{figure}[h]
                \centering
                \includegraphics[width=0.65\textwidth]{Images/NTP Config.jpg}
                \caption{NTP Configuration}
                \label{fig: NTP Configuration}
                \end{figure}
    
                \begin{itemize}
                    \item Restart the ntp service
                        \begin{itemize}
                            \item On linux: 
\begin{verbatim}
>sudo timedatectl set-timezone UTC
\end{verbatim} 
                            \item On Create3: 
                            
                            \begin{figure}[h]
                            \centering
                            \includegraphics[width=0.65\textwidth]{Images/Restart NTP Create3.jpg}
                            \caption{Restart NTP Create3}
                            \label{fig: Restart NTP Create3}
                            \end{figure}
                        \end{itemize}

\newpage

                    \item Possible error \\
                    If the RaspberryPI does not synchronize automatically when the robot is restarted, run the following command to do so manually.
\begin{verbatim}
>sudo timedatectl set-timezone UTC && sudo ntpdate [ip_du_relai]
\end{verbatim}
                    and enter this command in ~/.bashrc.
                 
                \end{itemize}
            \end{itemize}
\end{itemize}
